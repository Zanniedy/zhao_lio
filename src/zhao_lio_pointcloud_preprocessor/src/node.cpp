#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "zhao_lio_pointcloud_preprocessor/dynamic_filter.hpp"
#include "zhao_lio_pointcloud_preprocessor/optimizer.hpp"

namespace zhao_lio_pointcloud_preprocessor {

class PointCloudPreprocessorNode : public rclcpp::Node {
 public:
  PointCloudPreprocessorNode()
      : Node("zhao_lio_pointcloud_preprocessor"),
        input_topic_(declare_parameter<std::string>("input_topic", "/input/lidar")),
        output_topic_(declare_parameter<std::string>("output_topic", "/zhao_lio_x5/points/optimized")),
        optimizer_(OptimizerConfig{
            declare_parameter<double>("min_range", 0.3),
            declare_parameter<double>("max_range", 100.0),
            declare_parameter<double>("voxel_size", 0.10),
            declare_parameter<bool>("height_filter_enabled", false),
            declare_parameter<double>("min_z", -0.10),
            declare_parameter<double>("min_intensity", 0.0),
            declare_parameter<double>("max_z", 4.0)}) {
    height_filter_enabled_ = get_parameter("height_filter_enabled").as_bool();
    min_intensity_ = get_parameter("min_intensity").as_double();
    height_frame_ = declare_parameter<std::string>("height_frame", "base_link");
    if (height_frame_.empty()) throw std::invalid_argument("height_frame must not be empty");
    DynamicConfig config;
    config.enabled = declare_parameter<bool>("dynamic.enabled", config.enabled);
    config.min_dt = declare_parameter<double>("dynamic.min_dt", config.min_dt);
    config.max_dt = declare_parameter<double>("dynamic.max_dt", config.max_dt);
    config.max_scan_duration = declare_parameter<double>("dynamic.max_scan_duration", config.max_scan_duration);
    config.max_imu_gap = declare_parameter<double>("dynamic.max_imu_gap", config.max_imu_gap);
    config.correspondence_distance = declare_parameter<double>("dynamic.correspondence_distance", config.correspondence_distance);
    config.static_distance = declare_parameter<double>("dynamic.static_distance", config.static_distance);
    config.registration_inlier_distance = declare_parameter<double>(
        "dynamic.registration_inlier_distance", config.registration_inlier_distance);
    config.max_registration_rmse = declare_parameter<double>("dynamic.max_registration_rmse", config.max_registration_rmse);
    config.min_inlier_ratio = declare_parameter<double>("dynamic.min_inlier_ratio", config.min_inlier_ratio);
    config.max_ego_speed = declare_parameter<double>("dynamic.max_ego_speed", config.max_ego_speed);
    config.angular_bin_deg = declare_parameter<double>("dynamic.angular_bin_deg", config.angular_bin_deg);
    config.free_space_margin = declare_parameter<double>("dynamic.free_space_margin", config.free_space_margin);
    config.cluster_radius = declare_parameter<double>("dynamic.cluster_radius", config.cluster_radius);
    config.max_object_speed = declare_parameter<double>("dynamic.max_object_speed", config.max_object_speed);
    config.max_removed_ratio = declare_parameter<double>("dynamic.max_removed_ratio", config.max_removed_ratio);
    config.min_points = declare_parameter<int>("dynamic.min_points", config.min_points);
    config.min_cluster_points = declare_parameter<int>("dynamic.min_cluster_points", config.min_cluster_points);
    config.min_visibility_bins = declare_parameter<int>("dynamic.min_visibility_bins", config.min_visibility_bins);
    config.confirmation_frames = declare_parameter<int>("dynamic.confirmation_frames", config.confirmation_frames);
    config.cluster_tracking_enabled = declare_parameter<bool>("dynamic.cluster_tracking_enabled", false);
    config.cluster.tolerance = declare_parameter<double>("cluster.tolerance", config.cluster.tolerance);
    config.cluster.ground_z = declare_parameter<double>("cluster.ground_z", config.cluster.ground_z);
    config.cluster.ground_clearance = declare_parameter<double>("cluster.ground_clearance", config.cluster.ground_clearance);
    config.cluster.max_extent = declare_parameter<double>("cluster.max_extent", config.cluster.max_extent);
    config.cluster.association_distance = declare_parameter<double>("cluster.association_distance", config.cluster.association_distance);
    config.cluster.association_ambiguity = declare_parameter<double>("cluster.association_ambiguity", config.cluster.association_ambiguity);
    config.cluster.motion_window = declare_parameter<double>("cluster.motion_window", config.cluster.motion_window);
    config.cluster.min_displacement = declare_parameter<double>("cluster.min_displacement", config.cluster.min_displacement);
    config.cluster.min_speed = declare_parameter<double>("cluster.min_speed", config.cluster.min_speed);
    config.cluster.max_speed = declare_parameter<double>("cluster.max_speed", config.cluster.max_speed);
    config.cluster.fit_distance = declare_parameter<double>("cluster.fit_distance", config.cluster.fit_distance);
    config.cluster.max_fit_rmse = declare_parameter<double>("cluster.max_fit_rmse", config.cluster.max_fit_rmse);
    config.cluster.min_fit_improvement = declare_parameter<double>("cluster.min_fit_improvement", config.cluster.min_fit_improvement);
    config.cluster.min_overlap = declare_parameter<double>("cluster.min_overlap", config.cluster.min_overlap);
    config.cluster.track_timeout = declare_parameter<double>("cluster.track_timeout", config.cluster.track_timeout);
    config.cluster.min_points = declare_parameter<int>("cluster.min_points", config.cluster.min_points);
    config.cluster.confirmation_frames = declare_parameter<int>("cluster.confirmation_frames", config.cluster.confirmation_frames);
    cluster_tracking_enabled_ = config.cluster_tracking_enabled;
    dynamic_enabled_ = config.enabled;
    dynamic_filter_ = std::make_unique<DynamicFilter>(config);
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "mid70_link");
    wait_timeout_ = declare_parameter<double>("dynamic.imu_wait_timeout", 0.20);
    if (!std::isfinite(wait_timeout_) || wait_timeout_ <= 0.0 || wait_timeout_ > 2.0) {
      throw std::invalid_argument("dynamic.imu_wait_timeout must be in (0, 2] seconds");
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    if (declare_parameter<bool>("publish_calibrated_tf", false)) publish_calibration();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
    removed_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/zhao_lio_x5/points/dynamic", rclcpp::SensorDataQoS());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        "/zhao_lio_x5/dynamic_filter/status", rclcpp::QoS(1).transient_local());
    basic_status_publisher_ = create_publisher<std_msgs::msg::String>(
        "/zhao_lio_x5/point_filter/status", rclcpp::QoS(1).transient_local());
    cluster_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/zhao_lio_x5/dynamic_filter/clusters", rclcpp::QoS(1).transient_local());
    if (dynamic_enabled_) {
      imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
          declare_parameter<std::string>("imu_topic", "/input/imu"), rclcpp::SensorDataQoS().keep_last(200),
          [this](sensor_msgs::msg::Imu::ConstSharedPtr message) { on_imu(*message); });
    }
    subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          try {
            std::optional<Eigen::Isometry3d> cloud_to_height;
            if (height_filter_enabled_ || cluster_tracking_enabled_) {
              if (message->header.frame_id == height_frame_) {
                cloud_to_height = Eigen::Isometry3d::Identity();
              } else {
                try {
                  const auto transform = tf_buffer_->lookupTransform(
                      height_frame_, message->header.frame_id, rclcpp::Time(message->header.stamp));
                  const auto& quaternion = transform.transform.rotation;
                  Eigen::Quaterniond rotation(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
                  if (!rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1.0) > 1e-3) {
                    throw std::runtime_error("invalid height TF rotation");
                  }
                  Eigen::Isometry3d converted = Eigen::Isometry3d::Identity();
                  converted.linear() = rotation.normalized().toRotationMatrix();
                  const auto& translation = transform.transform.translation;
                  converted.translation() = Eigen::Vector3d(translation.x, translation.y, translation.z);
                  if (!converted.matrix().allFinite()) throw std::runtime_error("invalid height TF translation");
                  cloud_to_height = converted;
                } catch (const std::exception& error) {
                  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                      "height/cluster filters skipped: TF %s <- %s unavailable: %s",
                      height_frame_.c_str(), message->header.frame_id.c_str(), error.what());
                }
              }
            }
            OptimizerStats stats;
            auto optimized = optimizer_.optimize(*message, cloud_to_height, &stats);
            if (min_intensity_ > 0.0 && !stats.intensity_applied) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                  "intensity filter skipped: PointCloud2 has no intensity field");
            }
            std::ostringstream text;
            text << "input=" << static_cast<std::size_t>(message->width) * message->height
                 << " kept=" << optimized.width << " low_height=" << stats.low_height
                 << " low_intensity=" << stats.low_intensity << " height_applied=" << stats.height_applied
                 << " intensity_applied=" << stats.intensity_applied << " high_height=" << stats.high_height;
            std_msgs::msg::String basic_status;
            basic_status.data = text.str();
            basic_status_publisher_->publish(basic_status);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s", basic_status.data.c_str());
            if (!dynamic_enabled_) {
              publish(dynamic_filter_->bypass(optimized, "disabled"));
              return;
            }
            const double header_time = rclcpp::Time(optimized.header.stamp).seconds();
            if (header_time <= last_cloud_time_) {
              pending_.clear();
              dynamic_filter_->reset();
            }
            last_cloud_time_ = header_time;
            if (optimized.header.frame_id != lidar_frame_) {
              flush_pending("frame_changed");
              publish(dynamic_filter_->bypass(optimized, "unexpected_cloud_frame"));
              return;
            }
            double end;
            try {
              end = dynamic_filter_->scan_end(optimized);
            } catch (const std::exception& error) {
              flush_pending("invalid_timing");
              publish(dynamic_filter_->bypass(optimized, std::string("invalid_timing: ") + error.what()));
              return;
            }
            if (pending_.size() >= 5) {
              publish(dynamic_filter_->bypass(pending_.front().cloud, "queue_limit"));
              pending_.pop_front();
            }
            pending_.push_back({std::move(optimized), end, std::chrono::steady_clock::now(), cloud_to_height});
            drain();
          } catch (const std::exception& error) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "point cloud optimization failed: %s", error.what());
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { drain(); });
    RCLCPP_INFO(get_logger(), "point cloud optimizer: %s -> %s", input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void publish_calibration() {
    const auto rotation = declare_parameter<std::vector<double>>("extrinsic_R", std::vector<double>{});
    const auto translation = declare_parameter<std::vector<double>>("extrinsic_T", std::vector<double>{});
    if (rotation.size() != 9 || translation.size() != 3) {
      throw std::invalid_argument("publish_calibrated_tf requires LIO extrinsic_R and extrinsic_T");
    }
    Eigen::Matrix3d matrix;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) matrix(row, column) = rotation[row * 3 + column];
    }
    if (!matrix.allFinite() || (matrix.transpose() * matrix - Eigen::Matrix3d::Identity()).norm() > 1e-4 ||
        std::abs(matrix.determinant() - 1.0) > 1e-4 ||
        !std::all_of(translation.begin(), translation.end(), [](double value) { return std::isfinite(value); })) {
      throw std::invalid_argument("invalid LIO extrinsic transform");
    }
    const Eigen::Quaterniond quaternion(matrix);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = declare_parameter<std::string>("body_frame", "imu_link");
    transform.child_frame_id = lidar_frame_;
    if (transform.header.frame_id == transform.child_frame_id) throw std::invalid_argument("identical TF parent/child");
    transform.transform.translation.x = translation[0];
    transform.transform.translation.y = translation[1];
    transform.transform.translation.z = translation[2];
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    static_broadcaster_->sendTransform(transform);
    tf_buffer_->setTransform(transform, "lio_calibration", true);
  }

  void on_imu(const sensor_msgs::msg::Imu& message) {
    const double time = rclcpp::Time(message.header.stamp).seconds();
    if (time < last_imu_time_ || (!imu_frame_.empty() && imu_frame_ != message.header.frame_id)) {
      pending_.clear();
      dynamic_filter_->reset();
      last_cloud_time_ = -1.0;
    }
    last_imu_time_ = time;
    imu_frame_ = message.header.frame_id;
    try {
      if (message.angular_velocity_covariance[0] < 0.0) throw std::runtime_error("IMU has no angular velocity");
      Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
      if (message.header.frame_id != lidar_frame_) {
        const auto transform = tf_buffer_->lookupTransform(lidar_frame_, message.header.frame_id,
                                                           rclcpp::Time(message.header.stamp));
        const auto& quaternion = transform.transform.rotation;
        rotation = Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        if (!rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1.0) > 1e-3) {
          throw std::runtime_error("invalid IMU TF rotation");
        }
        rotation.normalize();
      }
      const Eigen::Vector3d gyro(message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z);
      dynamic_filter_->add_imu(time, rotation * gyro);
      drain();
    } catch (const std::exception& error) {
      dynamic_filter_->reset();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "dynamic filter waiting for valid IMU/TF: %s", error.what());
    }
  }

  void drain() {
    while (!pending_.empty()) {
      auto& pending = pending_.front();
      const bool ready = dynamic_filter_->has_imu_until(pending.end);
      const double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - pending.arrival).count();
      if (!ready && waited < wait_timeout_) break;
      try {
        publish(ready ? dynamic_filter_->filter(pending.cloud, pending.lidar_to_base) :
                        dynamic_filter_->bypass(pending.cloud, "imu_wait_timeout"));
      } catch (const std::exception& error) {
        publish(dynamic_filter_->bypass(pending.cloud, std::string("filter_error: ") + error.what()));
      }
      pending_.pop_front();
    }
  }

  void flush_pending(const std::string& reason) {
    while (!pending_.empty()) {
      publish(dynamic_filter_->bypass(pending_.front().cloud, reason));
      pending_.pop_front();
    }
  }

  void publish(const DynamicResult& result) {
    publisher_->publish(result.kept);
    removed_publisher_->publish(result.removed);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = result.kept.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    std::size_t moving_clusters = 0;
    for (const auto& cluster : result.clusters) {
      visualization_msgs::msg::Marker box;
      box.header = result.kept.header;
      if (result.reference_time > 0.0) box.header.stamp = rclcpp::Time(static_cast<std::int64_t>(result.reference_time * 1e9));
      box.ns = "tracked_clusters";
      box.id = cluster.id;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      const Eigen::Vector3d center = 0.5 * (cluster.minimum + cluster.maximum);
      const Eigen::Vector3d extent = (cluster.maximum - cluster.minimum).cwiseMax(0.05);
      box.pose.position.x = center.x();
      box.pose.position.y = center.y();
      box.pose.position.z = center.z();
      box.pose.orientation.w = 1.0;
      box.scale.x = extent.x();
      box.scale.y = extent.y();
      box.scale.z = extent.z();
      box.color.a = 0.20F;
      box.color.r = cluster.removed ? 1.0F : 0.9F;
      box.color.g = cluster.removed ? 0.1F : 0.7F;
      box.color.b = 0.1F;
      if (cluster.state == "stationary") {
        box.color.r = 0.1F;
        box.color.g = 0.8F;
      }
      markers.markers.push_back(box);
      auto label = box;
      label.ns = "cluster_labels";
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose.position.z = cluster.maximum.z() + 0.2;
      label.scale.z = 0.18;
      label.color.a = 1.0F;
      std::ostringstream text;
      text.precision(2);
      text << "#" << cluster.id << " " << cluster.state << " " << std::fixed << cluster.speed
           << "m/s n=" << cluster.points << " votes=" << cluster.confirmations;
      if (cluster.moving && !cluster.removed) text << " kept:limit";
      label.text = text.str();
      markers.markers.push_back(label);
      if (cluster.moving) ++moving_clusters;
    }
    cluster_publisher_->publish(markers);
    std::ostringstream text;
    text << "reason=" << result.reason << " kept=" << result.kept.width * result.kept.height
         << " removed=" << result.removed.width * result.removed.height << " candidates=" << result.candidates
         << " dt=" << result.dt << " inliers=" << result.inlier_ratio << " rmse=" << result.registration_rmse
         << " unmatched=" << result.unmatched << " clustered=" << result.clustered
         << " confirmed=" << result.confirmed << " converged=" << result.registration_converged
         << " tracks=" << result.clusters.size() << " moving_clusters=" << moving_clusters
         << " rejected_components=" << result.rejected_components;
    std_msgs::msg::String status;
    status.data = text.str();
    status_publisher_->publish(status);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s", status.data.c_str());
  }

  struct Pending {
    sensor_msgs::msg::PointCloud2 cloud;
    double end;
    std::chrono::steady_clock::time_point arrival;
    std::optional<Eigen::Isometry3d> lidar_to_base;
  };
  std::deque<Pending> pending_;
  std::unique_ptr<DynamicFilter> dynamic_filter_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::string lidar_frame_;
  std::string height_frame_;
  bool height_filter_enabled_;
  bool cluster_tracking_enabled_;
  double min_intensity_;
  std::string imu_frame_;
  bool dynamic_enabled_;
  double wait_timeout_;
  double last_cloud_time_ = -1.0;
  double last_imu_time_ = -1.0;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string input_topic_;
  std::string output_topic_;
  PointCloudOptimizer optimizer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr removed_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr basic_status_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
};

}  // namespace zhao_lio_pointcloud_preprocessor

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<zhao_lio_pointcloud_preprocessor::PointCloudPreprocessorNode>());
  rclcpp::shutdown();
  return 0;
}
