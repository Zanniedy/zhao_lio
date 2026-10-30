#include "zhao_lio/eskf30.h"

#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

namespace zhao_lio {

using S = State30;

void State30::plus(const Vec& dx) {
  pos += dx.segment<3>(kPos);
  rot = rot * so3_exp(dx.segment<3>(kRot));
  R_LI = R_LI * so3_exp(dx.segment<3>(kRLI));
  t_LI += dx.segment<3>(kTLI);
  vel += dx.segment<3>(kVel);
  omg += dx.segment<3>(kOmg);
  acc += dx.segment<3>(kAcc);
  grav += dx.segment<3>(kGrav);
  bg += dx.segment<3>(kBg);
  ba += dx.segment<3>(kBa);
}

Eskf30::Eskf30(const EskfOptions& opt) : opt_(opt) {
  Q_.setZero();
  Q_.block<3, 3>(S::kVel, S::kVel).diagonal().setConstant(opt.vel_cov);
  Q_.block<3, 3>(S::kOmg, S::kOmg).diagonal().setConstant(opt.omg_cov);
  Q_.block<3, 3>(S::kAcc, S::kAcc).diagonal().setConstant(opt.acc_cov);
  Q_.block<3, 3>(S::kBg, S::kBg).diagonal().setConstant(opt.bg_cov);
  Q_.block<3, 3>(S::kBa, S::kBa).diagonal().setConstant(opt.ba_cov);
  reset_covariance();
}

// Initial values from small_point_lio Estimator::reset(). With a fixed extrinsic its rows and
// columns are zero, so no update can move it.
void Eskf30::reset_covariance() {
  P = Mat::Identity() * 0.01;
  P.block<3, 3>(S::kGrav, S::kGrav).diagonal().setConstant(1e-4);
  P.block<3, 3>(S::kBg, S::kBg).diagonal().setConstant(1e-3);
  P.block<3, 3>(S::kBa, S::kBa).diagonal().setConstant(1e-3);
  if (!opt_.extrinsic_est) {
    P.middleRows<6>(S::kRLI).setZero();
    P.middleCols<6>(S::kRLI).setZero();
  }
}

void Eskf30::predict_state(double t) {
  const double dt = t - t_state_;
  if (dt <= 0.0) return;
  t_state_ = t;
  x.pos += x.vel * dt;
  x.rot = x.rot * so3_exp(x.omg * dt);
  x.vel += (x.rot * x.acc + x.grav) * dt;
}

void Eskf30::predict_cov(double t) {
  const double dt = t - t_cov_;
  if (dt <= 0.0) return;
  t_cov_ = t;
  const Eigen::Vector3d seg = -x.omg * dt;
  Mat F = Mat::Identity();
  F.block<3, 3>(S::kPos, S::kVel).diagonal().setConstant(dt);
  F.block<3, 3>(S::kRot, S::kRot) = so3_exp(seg);
  F.block<3, 3>(S::kRot, S::kOmg) = so3_A(seg) * dt;
  F.block<3, 3>(S::kVel, S::kRot) = -x.rot * hat(x.acc) * dt;  // small_point_lio omits dt here
  F.block<3, 3>(S::kVel, S::kAcc) = x.rot * dt;
  F.block<3, 3>(S::kVel, S::kGrav).diagonal().setConstant(dt);
  P = F * P * F.transpose() + Q_ * (dt * dt);
}
// z = [gyro - ω - bg ; acc - a - ba]. Row k of H has +1 at (ω|a)_k and (bg|ba)_k.
// Saturated axes are skipped; the accelerometer uses flag k+3 (small_point_lio reads flag k).
bool Eskf30::update_imu(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc) {
  bool satu[6] = {false, false, false, false, false, false};
  Eigen::Matrix<double, 6, 1> z;
  z.head<3>() = gyro - x.omg - x.bg;
  z.tail<3>() = acc - x.acc - x.ba;
  if (opt_.check_satu) {
    for (int i = 0; i < 3; ++i) {
      if (std::abs(gyro[i]) >= opt_.satu_gyro) { satu[i] = true; z[i] = 0.0; }
      if (std::abs(acc[i]) >= opt_.satu_acc) { satu[i + 3] = true; z[i + 3] = 0.0; }
    }
  }
  auto col_a = [](int k) { return k < 3 ? S::kOmg + k : S::kAcc + k - 3; };
  auto col_b = [](int k) { return k < 3 ? S::kBg + k : S::kBa + k - 3; };

  Eigen::Matrix<double, S::kDim, 6> PHt = Eigen::Matrix<double, S::kDim, 6>::Zero();
  Eigen::Matrix<double, 6, S::kDim> HP = Eigen::Matrix<double, 6, S::kDim>::Zero();
  for (int k = 0; k < 6; ++k) {
    if (satu[k]) continue;
    PHt.col(k) = P.col(col_a(k)) + P.col(col_b(k));
    HP.row(k) = P.row(col_a(k)) + P.row(col_b(k));
  }
  Eigen::Matrix<double, 6, 6> Smat = Eigen::Matrix<double, 6, 6>::Zero();
  for (int k = 0; k < 6; ++k) {
    if (!satu[k]) Smat.col(k) = HP.col(col_a(k)) + HP.col(col_b(k));
    Smat(k, k) += k < 3 ? opt_.imu_meas_omg_cov : opt_.imu_meas_acc_cov;
  }
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(Smat);
  if (ldlt.info() != Eigen::Success) return false;
  const Eigen::Matrix<double, S::kDim, 6> K = PHt * ldlt.solve(Eigen::Matrix<double, 6, 6>::Identity());
  x.plus(K * z);
  P -= K * HP;
  return true;
}

// r = nᵀ(R(R_LI p + t_LI) + pos) + d, H = [nᵀ, (p_imu × Rᵀn)ᵀ, (p × R_LIᵀRᵀn)ᵀ, (Rᵀn)ᵀ], rank-1 update.
bool Eskf30::update_point(const Eigen::Vector3d& n, double d, const Eigen::Vector3d& p_lidar) {
  const Eigen::Vector3d p_imu = x.R_LI * p_lidar + x.t_LI;
  const double r = n.dot(x.rot * p_imu + x.pos) + d;
  const Eigen::Vector3d C = x.rot.transpose() * n;
  Eigen::Matrix<double, 1, 12> H;
  if (opt_.extrinsic_est) {
    const Eigen::Vector3d B = p_lidar.cross(x.R_LI.transpose() * C);
    H << n.transpose(), p_imu.cross(C).transpose(), B.transpose(), C.transpose();
  } else {
    H << n.transpose(), p_imu.cross(C).transpose(), Eigen::Matrix<double, 1, 6>::Zero();
  }
  const Vec PHt = P.leftCols<12>() * H.transpose();
  const double s = H.dot(PHt.head<12>()) + opt_.laser_point_cov;
  if (s < 1e-12) return false;
  const Vec K = PHt / s;
  x.plus(K * (-r));
  P -= K * (H * P.topRows<12>());
  return true;
}

}  // namespace zhao_lio
