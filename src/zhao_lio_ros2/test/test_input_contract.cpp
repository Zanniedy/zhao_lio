#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_field.hpp>
#include "zhao_lio_ros2/msg_conversions.h"

TEST(InputContract, HostSecondsAndAlignedPointLayout) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "mid70_link";
  cloud.header.stamp.sec = 1700000000;
  cloud.height = 1;
  cloud.width = 2;
  cloud.point_step = 32;
  cloud.row_step = 64;
  cloud.is_bigendian = false;
  cloud.data.resize(cloud.row_step);
  auto add_field = [&](const std::string& name, uint32_t offset, uint8_t datatype) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    cloud.fields.push_back(field);
  };
  add_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
  add_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
  add_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
  add_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
  add_field("offset_time", 16, sensor_msgs::msg::PointField::UINT32);
  add_field("tag", 20, sensor_msgs::msg::PointField::UINT8);
  add_field("line", 21, sensor_msgs::msg::PointField::UINT8);
  add_field("timestamp", 24, sensor_msgs::msg::PointField::FLOAT64);
  sensor_msgs::PointCloud2Iterator<float> position(cloud, "x");
  sensor_msgs::PointCloud2Iterator<double> timestamp(cloud, "timestamp");
  for (int index = 0; index < 2; ++index, ++position, ++timestamp) {
    *position = static_cast<float>(index + 1);
    *timestamp = 1700000000.0 + index * 0.09;
  }
  zhao_lio::ScanFrame scan;
  ASSERT_TRUE(zhao_lio_ros2::scan_from_pointcloud2(cloud, scan));
  ASSERT_EQ(scan.points.size(), 2U);
  EXPECT_DOUBLE_EQ(scan.points[0].time, 1700000000.0);
  EXPECT_NEAR(scan.points[1].time - scan.points[0].time, 0.09, 1e-6);
  EXPECT_FLOAT_EQ(scan.points[1].xyz.x(), 2.0F);
}

TEST(InputContract, ChassisImuKeepsSIUnits) {
  sensor_msgs::msg::Imu message;
  message.header.stamp.sec = 1700000000;
  message.linear_acceleration.z = 9.81;
  message.angular_velocity.z = 0.2;
  const auto sample = zhao_lio_ros2::imu_from_msg(message, 1.0);
  EXPECT_DOUBLE_EQ(sample.acc.z(), 9.81);
}
