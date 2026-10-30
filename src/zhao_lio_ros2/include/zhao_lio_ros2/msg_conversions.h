#pragma once

#include <algorithm>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#ifdef HAVE_LIVOX_DRIVER
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

#include "zhao_lio/types.h"

namespace zhao_lio_ros2 {

inline zhao_lio::ImuSample imu_from_msg(const sensor_msgs::msg::Imu& m, double acc_scale) {
  zhao_lio::ImuSample s;
  s.time = rclcpp::Time(m.header.stamp).seconds();
  s.gyro << m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z;
  s.acc << m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z;
  s.acc *= acc_scale;
  return s;
}

inline void finish_scan(zhao_lio::ScanFrame& f, double header_time) {
  f.t_begin = header_time;
  f.t_end = header_time;
  for (const auto& p : f.points) f.t_end = std::max(f.t_end, p.time);
}

#ifdef HAVE_LIVOX_DRIVER
// Keeps first/strongest returns like Point-LIO's Livox handler ((tag & 0x30) is 0x00 or 0x10).
// Point times are header.stamp + offset_time, as in Point-LIO: `timebase` can be device time
// (seconds since power-on) when the LiDAR is not time-synchronized, while the IMU uses ROS time.
inline zhao_lio::ScanFrame scan_from_livox(const livox_ros_driver2::msg::CustomMsg& m) {
  zhao_lio::ScanFrame f;
  f.points.reserve(m.points.size());
  const double base = rclcpp::Time(m.header.stamp).seconds();
  for (const auto& pt : m.points) {
    const int ret = pt.tag & 0x30;
    if (ret != 0x00 && ret != 0x10) continue;
    zhao_lio::LidarPoint p;
    p.xyz = Eigen::Vector3f(pt.x, pt.y, pt.z);
    p.time = base + static_cast<double>(pt.offset_time) * 1e-9;
    f.points.push_back(p);
  }
  finish_scan(f, base);
  return f;
}
#endif
// livox_ros_driver2 PointCloud2 (xfer_format 0): float32 x/y/z and a float64 per-point
// "timestamp". Values above 1e12 are treated as nanoseconds, otherwise seconds.
// Returns false when the cloud has no "timestamp" field.
inline bool scan_from_pointcloud2(const sensor_msgs::msg::PointCloud2& m, zhao_lio::ScanFrame& f) {
  const bool has_ts = std::any_of(m.fields.begin(), m.fields.end(), [](const auto& fd) { return fd.name == "timestamp"; });
  if (!has_ts) return false;
  f.points.clear();
  f.points.reserve(static_cast<size_t>(m.width) * m.height);
  sensor_msgs::PointCloud2ConstIterator<float> x(m, "x"), y(m, "y"), z(m, "z");
  sensor_msgs::PointCloud2ConstIterator<double> ts(m, "timestamp");
  for (; x != x.end(); ++x, ++y, ++z, ++ts) {
    zhao_lio::LidarPoint p;
    p.xyz = Eigen::Vector3f(*x, *y, *z);
    p.time = *ts > 1e12 ? *ts * 1e-9 : *ts;
    f.points.push_back(p);
  }
  finish_scan(f, rclcpp::Time(m.header.stamp).seconds());
  return true;
}

}  // namespace zhao_lio_ros2
