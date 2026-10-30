#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace zhao_lio {

struct LidarPoint {
  Eigen::Vector3f xyz = Eigen::Vector3f::Zero();  // LiDAR frame
  double time = 0.0;                              // absolute time [s]
};

struct ImuSample {
  double time = 0.0;
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();  // [rad/s]
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();   // [m/s^2], already scaled by the front end
};

struct ScanFrame {
  double t_begin = 0.0;
  double t_end = 0.0;
  std::vector<LidarPoint> points;
};

// One LiDAR scan plus every IMU sample with time in (previous scan end, scan.t_end].
struct Measure {
  ScanFrame scan;
  std::vector<ImuSample> imus;
};

struct Event {
  enum Type : uint8_t { kPoint = 0, kImu = 1 };
  Type type = kPoint;
  double time = 0.0;
  uint32_t index = 0;  // index into the point or IMU vector
};

}  // namespace zhao_lio
