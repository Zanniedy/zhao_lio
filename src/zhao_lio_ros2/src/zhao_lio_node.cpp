#include <chrono>
#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "zhao_lio/zhao_lio.h"
#include "zhao_lio/io.h"
#include "zhao_lio/sync.h"
#include "zhao_lio_ros2/backend_runner.h"
#include "zhao_lio_ros2/config.h"
#include "zhao_lio_ros2/map_saver.h"
#include "zhao_lio_ros2/msg_conversions.h"

namespace zhao_lio_ros2 {

class ZhaoLioNode : public rclcpp::Node {
 public:
  // Saves the map and logs the result; also called once on shutdown.
  void save_map(const char* why) {
    if (!saver_.enabled() || frames_ == 0) return;
    const long n = saver_.save(*lio_);
    if (n >= 0) RCLCPP_INFO(get_logger(), "map saved (%s): %ld points -> %s", why, n, saver_.path().c_str());
    else RCLCPP_WARN(get_logger(), "map not saved (%s): %s", why, saver_.error().c_str());
  }

  explicit ZhaoLioNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
      : Node("zhao_lio", opt), cfg_(load_config(*this)), saver_(cfg_.map_save_dir) {
    lio_ = std::make_unique<zhao_lio::ZhaoLio>(cfg_.params);
    const BackendNodeConfig bcfg = load_backend_config(*this);
    if (bcfg.enabled) backend_ = std::make_unique<BackendRunner>(*this, bcfg, cfg_.odom_frame);
    if (!cfg_.traj_path.empty()) tum_.open(cfg_.traj_path);
    if (!cfg_.timing_csv.empty()) csv_.open(cfg_.timing_csv);

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/zhao_lio/odom", 100);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/zhao_lio/path", 10);
    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/zhao_lio/cloud_world", 10);
    // The voxel map used for matching, republished periodically for visualization.
    pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("/zhao_lio/map", rclcpp::QoS(1).transient_local());
    if (cfg_.map_publish_period > 0) {
      map_timer_ = create_wall_timer(std::chrono::duration<double>(cfg_.map_publish_period), [this] { publish_map(); });
    }
    srv_save_ = create_service<std_srvs::srv::Trigger>("/zhao_lio/save_map",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          const long n = saver_.save(*lio_);
          res->success = n >= 0;
          res->message = n >= 0 ? std::to_string(n) + " points -> " + saver_.path() : saver_.error();
        });
    if (saver_.enabled() && cfg_.map_autosave_period > 0) {
      autosave_timer_ = create_wall_timer(std::chrono::duration<double>(cfg_.map_autosave_period),
                                          [this] { save_map("autosave"); });
    }
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    path_.header.frame_id = cfg_.odom_frame;

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(cfg_.imu_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu& m) { sync_.push_imu(imu_from_msg(m, cfg_.acc_scale)); spin_measures(); });

    if (cfg_.lidar_type == "pointcloud2") {
      sub_pc2_ = create_subscription<sensor_msgs::msg::PointCloud2>(cfg_.lidar_topic, rclcpp::SensorDataQoS(),
          [this](const sensor_msgs::msg::PointCloud2& m) {
            zhao_lio::ScanFrame f;
            if (!scan_from_pointcloud2(m, f)) {
              RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "PointCloud2 has no 'timestamp' field");
              return;
            }
            sync_.push_scan(std::move(f));
            spin_measures();
          });
    } else if (cfg_.lidar_type == "livox_custom") {
#ifdef HAVE_LIVOX_DRIVER
      sub_livox_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(cfg_.lidar_topic, rclcpp::SensorDataQoS(),
          [this](const livox_ros_driver2::msg::CustomMsg& m) { sync_.push_scan(scan_from_livox(m)); spin_measures(); });
#else
      throw std::runtime_error("lidar_type livox_custom needs livox_ros_driver2 at build time");
#endif
    } else {
      throw std::invalid_argument("unknown lidar_type: " + cfg_.lidar_type);
    }
    RCLCPP_INFO(get_logger(), "zhao_lio started, lidar=%s (%s), imu=%s", cfg_.lidar_topic.c_str(),
                cfg_.lidar_type.c_str(), cfg_.imu_topic.c_str());
  }
 private:
  void spin_measures() {
    zhao_lio::Measure m;
    while (sync_.pop(m)) {
      if (backend_ && backend_->consume_reset()) {
        RCLCPP_WARN(get_logger(), "back end requested a front-end restart");
        lio_->reset();
      }
      if (!lio_->process(m, res_)) continue;
      tum_.write(res_.time, res_.state);
      csv_.write(res_);
      if (backend_) backend_->push(res_, lio_->session());
      publish(res_);
      if (++frames_ % 100 == 0) {
        RCLCPP_INFO(get_logger(), "frame %d: total %.2f ms (pre %.2f ds %.2f match %.2f upd %.2f map %.2f) used %d/%zu rematch %d",
                    frames_, res_.timing.total_ms, res_.timing.preprocess_ms, res_.timing.downsample_ms,
                    res_.timing.prematch_ms, res_.timing.update_ms, res_.timing.map_ms, res_.stats.points_used,
                    res_.num_downsampled, res_.stats.rematched);
      }
    }
  }

  static void fill_pose(const zhao_lio::State30& s, geometry_msgs::msg::Pose& pose) {
    const Eigen::Quaterniond q(s.rot);
    pose.position.x = s.pos.x(); pose.position.y = s.pos.y(); pose.position.z = s.pos.z();
    pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
  }

  void publish_odom(double t, const zhao_lio::State30& s) {
    nav_msgs::msg::Odometry od;
    od.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
    od.header.frame_id = cfg_.odom_frame;
    od.child_frame_id = cfg_.body_frame;
    fill_pose(s, od.pose.pose);
    const Eigen::Vector3d v_body = s.rot.transpose() * s.vel;
    od.twist.twist.linear.x = v_body.x(); od.twist.twist.linear.y = v_body.y(); od.twist.twist.linear.z = v_body.z();
    od.twist.twist.angular.x = s.omg.x(); od.twist.twist.angular.y = s.omg.y(); od.twist.twist.angular.z = s.omg.z();
    pub_odom_->publish(od);
  }

  static void fill_cloud(const std::vector<Eigen::Vector3f>& pts, sensor_msgs::msg::PointCloud2& cloud) {
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const auto& p : pts) { *x = p.x(); *y = p.y(); *z = p.z(); ++x; ++y; ++z; }
  }

  void publish_map() {
    if (frames_ == 0 || pub_map_->get_subscription_count() == 0) return;
    lio_->map().get_points(map_points_);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = rclcpp::Time(static_cast<int64_t>(res_.time * 1e9));
    cloud.header.frame_id = cfg_.odom_frame;
    fill_cloud(map_points_, cloud);
    pub_map_->publish(cloud);
  }

  void publish(const zhao_lio::ScanResult& r) {
    for (const auto& ts : r.state_trace) publish_odom(ts.first, ts.second);  // only when publish_pointwise_odom
    publish_odom(r.time, r.state);
    const rclcpp::Time stamp(static_cast<int64_t>(r.time * 1e9));

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = cfg_.odom_frame;
    tf.child_frame_id = cfg_.body_frame;
    tf.transform.translation.x = r.state.pos.x(); tf.transform.translation.y = r.state.pos.y(); tf.transform.translation.z = r.state.pos.z();
    const Eigen::Quaterniond q(r.state.rot);
    tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y(); tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
    if (cfg_.publish_tf) tf_->sendTransform(tf);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = tf.header;
    fill_pose(r.state, ps.pose);
    path_.header.stamp = stamp;
    path_.poses.push_back(ps);
    pub_path_->publish(path_);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = tf.header;
    fill_cloud(r.points_world, cloud);
    pub_cloud_->publish(cloud);
  }
  NodeConfig cfg_;
  MapSaver saver_;
  std::unique_ptr<BackendRunner> backend_;
  std::unique_ptr<zhao_lio::ZhaoLio> lio_;
  zhao_lio::MeasureSync sync_;
  zhao_lio::ScanResult res_;
  zhao_lio::TumWriter tum_;
  zhao_lio::TimingCsv csv_;
  nav_msgs::msg::Path path_;
  int frames_ = 0;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_, pub_map_;
  rclcpp::TimerBase::SharedPtr map_timer_, autosave_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
  std::vector<Eigen::Vector3f> map_points_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2_;
#ifdef HAVE_LIVOX_DRIVER
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox_;
#endif
};

}  // namespace zhao_lio_ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<zhao_lio_ros2::ZhaoLioNode>();
  rclcpp::spin(node);
  node->save_map("exit");  // spin returns on Ctrl-C
  rclcpp::shutdown();
  return 0;
}
