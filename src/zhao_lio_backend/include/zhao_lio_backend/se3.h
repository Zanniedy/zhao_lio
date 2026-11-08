#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

// SE(3) helpers. Tangent vectors are ordered [rho (translation); phi (rotation)], and every
// perturbation is on the right: T ⊕ xi = T * Exp(xi).
namespace zhao_lio_backend {

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Pose = Eigen::Isometry3d;

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return m;
}

inline Eigen::Matrix3d so3_exp(const Eigen::Vector3d& phi) {
  const double th = phi.norm();
  if (th < 1e-10) return Eigen::Matrix3d::Identity() + skew(phi);
  return Eigen::AngleAxisd(th, phi / th).toRotationMatrix();
}

inline Eigen::Vector3d so3_log(const Eigen::Matrix3d& R) {
  const Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}

// Left Jacobian of SO(3); V in Exp([rho; phi]) = (Exp(phi), V(phi) rho).
inline Eigen::Matrix3d so3_left_jacobian(const Eigen::Vector3d& phi) {
  const double th = phi.norm();
  const Eigen::Matrix3d K = skew(phi);
  if (th < 1e-6) return Eigen::Matrix3d::Identity() + 0.5 * K + K * K / 6.0;
  const double th2 = th * th;
  return Eigen::Matrix3d::Identity() + (1 - std::cos(th)) / th2 * K + (th - std::sin(th)) / (th2 * th) * K * K;
}

inline Eigen::Matrix3d so3_left_jacobian_inv(const Eigen::Vector3d& phi) {
  const double th = phi.norm();
  const Eigen::Matrix3d K = skew(phi);
  if (th < 1e-6) return Eigen::Matrix3d::Identity() - 0.5 * K + K * K / 12.0;
  return Eigen::Matrix3d::Identity() - 0.5 * K + (1.0 / (th * th) - (1 + std::cos(th)) / (2 * th * std::sin(th))) * K * K;
}

inline Pose se3_exp(const Vec6& xi) {
  Pose T = Pose::Identity();
  const Eigen::Vector3d phi = xi.tail<3>();
  T.linear() = so3_exp(phi);
  T.translation() = so3_left_jacobian(phi) * xi.head<3>();
  return T;
}

inline Vec6 se3_log(const Pose& T) {
  Vec6 xi;
  const Eigen::Vector3d phi = so3_log(T.linear());
  xi.tail<3>() = phi;
  xi.head<3>() = so3_left_jacobian_inv(phi) * T.translation();
  return xi;
}

// Adjoint for [rho; phi]: Exp(Ad_T xi) = T Exp(xi) T^-1.
inline Mat6 se3_adjoint(const Pose& T) {
  Mat6 A = Mat6::Zero();
  const Eigen::Matrix3d R = T.linear();
  A.block<3, 3>(0, 0) = R;
  A.block<3, 3>(0, 3) = skew(T.translation()) * R;
  A.block<3, 3>(3, 3) = R;
  return A;
}

// Inverse right Jacobian of SE(3) to second order: Jr^-1(xi) ≈ I + ad(xi)/2 + ad(xi)^2/12.
inline Mat6 se3_right_jacobian_inv(const Vec6& xi) {
  Mat6 ad = Mat6::Zero();
  const Eigen::Matrix3d P = skew(xi.tail<3>());
  ad.block<3, 3>(0, 0) = P;
  ad.block<3, 3>(0, 3) = skew(xi.head<3>());
  ad.block<3, 3>(3, 3) = P;
  return Mat6::Identity() + 0.5 * ad + ad * ad / 12.0;
}

inline Pose pose_from(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
  Pose T = Pose::Identity();
  T.linear() = R;
  T.translation() = t;
  return T;
}

inline double yaw_of(const Eigen::Matrix3d& R) { return std::atan2(R(1, 0), R(0, 0)); }

// R = Rz(yaw) * R_tilt; returns R_tilt (roll and pitch only).
inline Eigen::Matrix3d tilt_of(const Eigen::Matrix3d& R) {
  return Eigen::AngleAxisd(-yaw_of(R), Eigen::Vector3d::UnitZ()).toRotationMatrix() * R;
}

inline Eigen::Matrix3d rot_z(double yaw) { return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }

// Re-orthonormalizes after many compositions.
inline Pose normalized(const Pose& T) {
  Pose out = T;
  out.linear() = Eigen::Quaterniond(T.linear()).normalized().toRotationMatrix();
  return out;
}

}  // namespace zhao_lio_backend
