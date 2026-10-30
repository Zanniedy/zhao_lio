#pragma once

#include <cmath>

#include <Eigen/Core>

namespace zhao_lio {

inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0, -v[2], v[1],
       v[2], 0, -v[0],
       -v[1], v[0], 0;
  return m;
}

inline Eigen::Matrix3d so3_exp(const Eigen::Vector3d& ang) {
  const double theta = ang.norm();
  if (theta < 1e-7) return Eigen::Matrix3d::Identity() + hat(ang);
  const Eigen::Matrix3d K = hat(ang / theta);
  return Eigen::Matrix3d::Identity() + std::sin(theta) * K + (1.0 - std::cos(theta)) * K * K;
}

// SO(3) Jacobian used by small_point_lio's covariance propagation (A_matrix).
inline Eigen::Matrix3d so3_A(const Eigen::Vector3d& v) {
  const double sq = v.squaredNorm();
  if (sq < 1e-22) return Eigen::Matrix3d::Identity();
  const double n = std::sqrt(sq);
  const Eigen::Matrix3d H = hat(v);
  return Eigen::Matrix3d::Identity() + (1.0 - std::cos(n)) / sq * H + (1.0 - std::sin(n) / n) / sq * H * H;
}

}  // namespace zhao_lio
