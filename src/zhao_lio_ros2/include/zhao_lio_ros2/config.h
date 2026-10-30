#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "zhao_lio/params.h"

namespace zhao_lio_ros2 {

struct NodeConfig {
  zhao_lio::Params params;
  std::string lidar_topic, imu_topic, lidar_type, odom_frame, body_frame, traj_path, timing_csv;
  double acc_scale = 1.0;  // raw IMU acc -> m/s^2
};

inline NodeConfig load_config(rclcpp::Node& n) {
  NodeConfig c;
  zhao_lio::Params& p = c.params;
  c.lidar_topic = n.declare_parameter<std::string>("lidar_topic", "/livox/lidar");
  c.imu_topic = n.declare_parameter<std::string>("imu_topic", "/livox/imu");
  c.lidar_type = n.declare_parameter<std::string>("lidar_type", "livox_custom");  // or "pointcloud2"
  c.odom_frame = n.declare_parameter<std::string>("odom_frame", "odom");
  c.body_frame = n.declare_parameter<std::string>("body_frame", "body");
  c.traj_path = n.declare_parameter<std::string>("traj_path", "");
  c.timing_csv = n.declare_parameter<std::string>("timing_csv", "");

  p.strategy = zhao_lio::parse_strategy(n.declare_parameter<std::string>("update_strategy", "pointwise_prematch"));
  p.num_threads = static_cast<int>(n.declare_parameter<int64_t>("num_threads", p.num_threads));
  p.blind_min = n.declare_parameter<double>("blind_min", p.blind_min);
  p.blind_max = n.declare_parameter<double>("blind_max", p.blind_max);
  p.point_filter_num = static_cast<int>(n.declare_parameter<int64_t>("point_filter_num", p.point_filter_num));
  p.downsample_size = static_cast<float>(n.declare_parameter<double>("downsample_size", p.downsample_size));
  p.downsample_parallel = n.declare_parameter<bool>("downsample_parallel", p.downsample_parallel);
  p.map_resolution = static_cast<float>(n.declare_parameter<double>("map_resolution", p.map_resolution));
  p.map_capacity = static_cast<size_t>(n.declare_parameter<int64_t>("map_capacity", static_cast<int64_t>(p.map_capacity)));
  p.init_map_frames = static_cast<int>(n.declare_parameter<int64_t>("init_map_frames", p.init_map_frames));
  p.plane_thr = n.declare_parameter<double>("plane_thr", p.plane_thr);
  p.match_s = n.declare_parameter<double>("match_s", p.match_s);
  p.rematch_thr = n.declare_parameter<double>("rematch_thr", p.rematch_thr);
  p.laser_point_cov = n.declare_parameter<double>("laser_point_cov", p.laser_point_cov);
  p.imu_meas_omg_cov = n.declare_parameter<double>("imu_meas_omg_cov", p.imu_meas_omg_cov);
  p.imu_meas_acc_cov = n.declare_parameter<double>("imu_meas_acc_cov", p.imu_meas_acc_cov);
  p.vel_cov = n.declare_parameter<double>("vel_cov", p.vel_cov);
  p.omg_cov = n.declare_parameter<double>("omg_cov", p.omg_cov);
  p.acc_cov = n.declare_parameter<double>("acc_cov", p.acc_cov);
  p.bg_cov = n.declare_parameter<double>("bg_cov", p.bg_cov);
  p.ba_cov = n.declare_parameter<double>("ba_cov", p.ba_cov);
  // IMU units: acc_norm is the raw magnitude of 1 g (1.0 for Livox, which reports g).
  p.gravity_norm = n.declare_parameter<double>("gravity_norm", p.gravity_norm);
  const double acc_norm = n.declare_parameter<double>("acc_norm", 1.0);
  c.acc_scale = p.gravity_norm / acc_norm;
  p.check_satu = n.declare_parameter<bool>("check_satu", p.check_satu);
  p.satu_gyro = n.declare_parameter<double>("satu_gyro", 35.0) * 0.99;                    // raw units = rad/s
  p.satu_acc = n.declare_parameter<double>("satu_acc", 3.0) * c.acc_scale * 0.99;          // raw units -> m/s^2
  p.imu_init_num = static_cast<int>(n.declare_parameter<int64_t>("imu_init_num", p.imu_init_num));

  p.extrinsic_est_en = n.declare_parameter<bool>("extrinsic_est_en", p.extrinsic_est_en);
  const auto T = n.declare_parameter<std::vector<double>>("extrinsic_T", {0.0, 0.0, 0.0});
  const auto R = n.declare_parameter<std::vector<double>>("extrinsic_R", {1, 0, 0, 0, 1, 0, 0, 0, 1});
  if (T.size() != 3 || R.size() != 9) throw std::invalid_argument("extrinsic_T needs 3 values, extrinsic_R needs 9");
  p.extrinsic_T = Eigen::Vector3d(T[0], T[1], T[2]);
  p.extrinsic_R << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];

  p.record_pointwise_states = n.declare_parameter<bool>("publish_pointwise_odom", false);
  return c;
}

}  // namespace zhao_lio_ros2
