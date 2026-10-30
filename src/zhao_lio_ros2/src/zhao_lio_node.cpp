#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "zhao_lio/zhao_lio.h"
#include "zhao_lio/io.h"
#include "zhao_lio/sync.h"
#include "zhao_lio_ros2/config.h"
#include "zhao_lio_ros2/msg_conversions.h"

namespace zhao_lio_ros2 {

class ZhaoLioNode : public rclcpp::Node {
 public:
  explicit ZhaoLioNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("zhao_lio", opt) {
    cfg_ = load_config(*this);
    lio_ = std::make_unique<zhao_lio::ZhaoLio>(cfg_.params);
    if (!cfg_.traj_path.empty()) tum_.open(cfg_.traj_path);
    if (!cfg_.timing_csv.empty()) csv_.open(cfg_.timing_csv);

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/zhao_lio/odom", 100);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/zhao_lio/path", 10);
    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/zhao_lio/cloud_world", 10);
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
      if (!lio_->process(m, res_)) continue;
      tum_.write(res_.time, res_.state);
      csv_.write(res_);
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
    tf_->sendTransform(tf);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = tf.header;
    fill_pose(r.state, ps.pose);
    path_.header.stamp = stamp;
    path_.poses.push_back(ps);
    pub_path_->publish(path_);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = tf.header;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(r.points_world.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const auto& p : r.points_world) { *x = p.x(); *y = p.y(); *z = p.z(); ++x; ++y; ++z; }
    pub_cloud_->publish(cloud);
  }
  NodeConfig cfg_;
  std::unique_ptr<zhao_lio::ZhaoLio> lio_;
  zhao_lio::MeasureSync sync_;
  zhao_lio::ScanResult res_;
  zhao_lio::TumWriter tum_;
  zhao_lio::TimingCsv csv_;
  nav_msgs::msg::Path path_;
  int frames_ = 0;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
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
  rclcpp::spin(std::make_shared<zhao_lio_ros2::ZhaoLioNode>());
  rclcpp::shutdown();
  return 0;
}
