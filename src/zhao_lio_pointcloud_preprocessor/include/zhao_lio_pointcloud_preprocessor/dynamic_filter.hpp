#pragma once

#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "zhao_lio_pointcloud_preprocessor/cluster_tracker.hpp"

namespace zhao_lio_pointcloud_preprocessor {

struct DynamicConfig {
  bool enabled = true;
  double min_dt = 0.04;
  double max_dt = 0.30;
  double max_scan_duration = 0.15;
  double max_imu_gap = 0.03;
  double correspondence_distance = 0.60;
  double static_distance = 0.12;
  double registration_inlier_distance = 0.30;
  double max_registration_rmse = 0.18;
  double min_inlier_ratio = 0.60;
  double max_ego_speed = 3.0;
  double angular_bin_deg = 1.0;
  double free_space_margin = 0.15;
  double cluster_radius = 0.35;
  double max_object_speed = 6.0;
  double max_removed_ratio = 0.15;
  int min_points = 100;
  int min_cluster_points = 3;
  int min_visibility_bins = 3;
  int confirmation_frames = 2;
  bool cluster_tracking_enabled = false;
  ClusterConfig cluster;
};

struct DynamicResult {
  sensor_msgs::msg::PointCloud2 kept;
  sensor_msgs::msg::PointCloud2 removed;
  std::string reason;
  double dt = 0.0;
  double reference_time = 0.0;
  double inlier_ratio = 0.0;
  double registration_rmse = 0.0;
  std::size_t candidates = 0;
  std::size_t unmatched = 0;
  std::size_t clustered = 0;
  std::size_t confirmed = 0;
  bool registration_converged = false;
  std::vector<ClusterView> clusters;
  std::size_t rejected_components = 0;
};

class DynamicFilter {
 public:
  explicit DynamicFilter(DynamicConfig config = {});
  bool add_imu(double time, const Eigen::Vector3d& gyro_lidar);
  bool has_imu_until(double time) const;
  double scan_end(const sensor_msgs::msg::PointCloud2& cloud) const;
  DynamicResult filter(const sensor_msgs::msg::PointCloud2& cloud,
                       const std::optional<Eigen::Isometry3d>& lidar_to_base = std::nullopt);
  DynamicResult bypass(const sensor_msgs::msg::PointCloud2& cloud, const std::string& reason);
  void reset();

 private:
  struct ImuPose {
    double time;
    Eigen::Vector3d gyro;
    Eigen::Quaterniond orientation;
  };
  struct Scan {
    double time = 0.0;
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    std::vector<Eigen::Vector3d> points;
    std::vector<int> confirmations;
  };
  bool orientation_at(double time, Eigen::Quaterniond& orientation) const;
  DynamicConfig config_;
  ClusterTracker cluster_tracker_;
  std::deque<ImuPose> imu_;
  Scan previous_;
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  std::string frame_;
};

}
