#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

namespace zhao_lio {

enum class StrategyType { kPointwise, kPointwisePrematch };

inline StrategyType parse_strategy(const std::string& s) {
  if (s == "pointwise") return StrategyType::kPointwise;
  if (s == "pointwise_prematch") return StrategyType::kPointwisePrematch;
  throw std::invalid_argument("unknown update_strategy: " + s);
}

struct Params {
  StrategyType strategy = StrategyType::kPointwisePrematch;
  int num_threads = 0;  // 0: TBB default

  // preprocess
  double blind_min = 0.5;
  double blind_max = 100.0;
  int point_filter_num = 1;

  // downsample (<= 0 disables)
  float downsample_size = 0.5f;
  bool downsample_parallel = false;

  // map
  float map_resolution = 0.5f;
  std::size_t map_capacity = 1000000;
  int init_map_frames = 3;

  // association
  double plane_thr = 0.1;
  double match_s = 81.0;
  double rematch_thr = 0.1;

  // filter noise (defaults from small_point_lio mid360.yaml)
  double laser_point_cov = 0.01;
  double imu_meas_omg_cov = 0.01;
  double imu_meas_acc_cov = 0.01;
  double vel_cov = 20.0;
  double omg_cov = 1000.0;
  double acc_cov = 500.0;
  double bg_cov = 1e-4;
  double ba_cov = 1e-4;

  // IMU (SI units; the front end converts)
  bool check_satu = true;
  double satu_gyro = 35.0 * 0.99;
  double satu_acc = 3.0 * 9.81 * 0.99;
  double gravity_norm = 9.81;
  int imu_init_num = 100;

  // extrinsic: p_imu = extrinsic_R * p_lidar + extrinsic_T
  bool extrinsic_est_en = false;
  Eigen::Matrix3d extrinsic_R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d extrinsic_T = Eigen::Vector3d::Zero();

  // output
  bool record_pointwise_states = false;
};

}  // namespace zhao_lio
