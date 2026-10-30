#pragma once

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Geometry>

#include "zhao_lio/types.h"

namespace zhao_lio::sim {

// Box room [-5,5]x[-5,5]x[-1.5,2.5]; LiDAR frame == IMU frame. Static until t_static, then
// x, y and yaw follow a·(1 - cos(w·τ)), τ = t - t_static.
struct Trajectory {
  double t_static = 1.5, ax = 1.0, ay = 0.6, ayaw = 0.5, w = 0.6, g = 9.81;

  double tau(double t) const { return std::max(0.0, t - t_static); }
  double c(double t) const { return std::cos(w * tau(t)); }
  double s(double t) const { return t > t_static ? std::sin(w * tau(t)) : 0.0; }
  Eigen::Vector3d pos(double t) const { return {ax * (1 - c(t)), ay * (1 - c(t)), 0.0}; }
  Eigen::Vector3d acc_world(double t) const {
    if (t <= t_static) return Eigen::Vector3d::Zero();
    return {ax * w * w * c(t), ay * w * w * c(t), 0.0};
  }
  double yaw(double t) const { return ayaw * (1 - c(t)); }
  double yaw_rate(double t) const { return ayaw * w * s(t); }
  Eigen::Matrix3d rot(double t) const { return Eigen::AngleAxisd(yaw(t), Eigen::Vector3d::UnitZ()).toRotationMatrix(); }
};

inline ImuSample make_imu(const Trajectory& tr, double t) {
  ImuSample s;
  s.time = t;
  s.gyro = Eigen::Vector3d(0, 0, tr.yaw_rate(t));
  s.acc = tr.rot(t).transpose() * (tr.acc_world(t) + Eigen::Vector3d(0, 0, tr.g));
  return s;
}

inline double ray_box(const Eigen::Vector3d& o, const Eigen::Vector3d& d) {
  const Eigen::Vector3d lo(-5, -5, -1.5), hi(5, 5, 2.5);
  double best = 1e9;
  for (int a = 0; a < 3; ++a) {
    if (std::abs(d[a]) < 1e-9) continue;
    const double s = ((d[a] > 0 ? hi[a] : lo[a]) - o[a]) / d[a];
    if (s > 0 && s < best) best = s;
  }
  return best;
}
// One scan of n points over (t0, t0 + dt]: elevation −30°..60°, golden-angle azimuth, 5 mm range noise.
inline ScanFrame make_scan(const Trajectory& tr, double t0, double dt, int n, std::mt19937& rng) {
  std::normal_distribution<double> noise(0.0, 0.005);
  const double golden = M_PI * (3.0 - std::sqrt(5.0));
  ScanFrame f;
  f.t_begin = t0;
  f.t_end = t0 + dt;
  f.points.reserve(n);
  for (int k = 0; k < n; ++k) {
    const double t = t0 + dt * (k + 1) / n;
    const double el = (-30.0 + 90.0 * std::fmod(k * 0.7548776662466927, 1.0)) * M_PI / 180.0;
    const double az = golden * k;
    const Eigen::Vector3d d_body(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
    const double range = ray_box(tr.pos(t), tr.rot(t) * d_body) + noise(rng);
    LidarPoint p;
    p.xyz = (d_body * range).cast<float>();
    p.time = t;
    f.points.push_back(p);
  }
  return f;
}

// Dense map points on the six walls (spacing 0.05 m), for building a map directly.
inline std::vector<Eigen::Vector3f> room_surface_points() {
  std::vector<Eigen::Vector3f> v;
  for (float a = -5; a <= 5; a += 0.05f)
    for (float b = -1.5f; b <= 2.5f; b += 0.05f) {
      v.emplace_back(-5, a, b); v.emplace_back(5, a, b);
      v.emplace_back(a, -5, b); v.emplace_back(a, 5, b);
    }
  for (float a = -5; a <= 5; a += 0.05f)
    for (float b = -5; b <= 5; b += 0.05f) { v.emplace_back(a, b, -1.5f); v.emplace_back(a, b, 2.5f); }
  return v;
}

}  // namespace zhao_lio::sim
