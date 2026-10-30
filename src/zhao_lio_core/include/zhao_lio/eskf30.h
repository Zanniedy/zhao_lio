#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "zhao_lio/so3.h"

namespace zhao_lio {

// Point-LIO "output" state: ω and a are states, IMU is a measurement.
struct State30 {
  static constexpr int kDim = 30;
  static constexpr int kPos = 0, kRot = 3, kRLI = 6, kTLI = 9, kVel = 12, kOmg = 15, kAcc = 18,
                       kGrav = 21, kBg = 24, kBa = 27;
  using Vec = Eigen::Matrix<double, kDim, 1>;

  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();   // IMU -> world
  Eigen::Matrix3d R_LI = Eigen::Matrix3d::Identity();  // LiDAR -> IMU
  Eigen::Vector3d t_LI = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d omg = Eigen::Vector3d::Zero();       // body angular velocity
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();       // body specific force
  Eigen::Vector3d grav = Eigen::Vector3d::Zero();      // world gravity
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();

  void plus(const Vec& dx);
  Eigen::Vector3d lidar_to_world(const Eigen::Vector3d& p_lidar) const { return rot * (R_LI * p_lidar + t_LI) + pos; }
};

struct EskfOptions {
  double laser_point_cov = 0.01;
  double imu_meas_omg_cov = 0.01;
  double imu_meas_acc_cov = 0.01;
  double vel_cov = 20.0, omg_cov = 1000.0, acc_cov = 500.0, bg_cov = 1e-4, ba_cov = 1e-4;
  bool check_satu = true;
  double satu_gyro = 35.0 * 0.99;      // rad/s
  double satu_acc = 3.0 * 9.81 * 0.99; // m/s^2
  bool extrinsic_est = false;
};

class Eskf30 {
 public:
  using Mat = Eigen::Matrix<double, State30::kDim, State30::kDim>;
  using Vec = State30::Vec;

  State30 x;
  Mat P;

  explicit Eskf30(const EskfOptions& opt = EskfOptions());
  void reset_covariance();
  void init_time(double t) { t_state_ = t; t_cov_ = t; }
  double time() const { return t_state_; }
  const EskfOptions& options() const { return opt_; }

  void predict_state(double t);  // nominal p, R, v only
  void predict_cov(double t);    // 30x30 covariance, call at IMU times
  bool update_imu(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc);
  double point_residual(const Eigen::Vector3d& n, double d, const Eigen::Vector3d& p_lidar) const {
    return n.dot(x.lidar_to_world(p_lidar)) + d;
  }
  bool update_point(const Eigen::Vector3d& n, double d, const Eigen::Vector3d& p_lidar);

 private:
  EskfOptions opt_;
  Mat Q_;
  double t_state_ = 0.0;
  double t_cov_ = 0.0;
};

}  // namespace zhao_lio
