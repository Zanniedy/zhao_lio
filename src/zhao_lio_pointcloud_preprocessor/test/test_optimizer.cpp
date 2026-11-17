#include <cstring>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "zhao_lio_pointcloud_preprocessor/optimizer.hpp"

namespace {

sensor_msgs::msg::PointCloud2 make_cloud() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "mid70_link";
  cloud.width = 4;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::msg::PointField x;
  x.name = "x";
  x.offset = 0;
  x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x.count = 1;
  sensor_msgs::msg::PointField y;
  y.name = "y";
  y.offset = 4;
  y.datatype = sensor_msgs::msg::PointField::FLOAT32;
  y.count = 1;
  sensor_msgs::msg::PointField z;
  z.name = "z";
  z.offset = 8;
  z.datatype = sensor_msgs::msg::PointField::FLOAT32;
  z.count = 1;
  sensor_msgs::msg::PointField timestamp_field;
  timestamp_field.name = "timestamp";
  timestamp_field.offset = 12;
  timestamp_field.datatype = sensor_msgs::msg::PointField::FLOAT64;
  timestamp_field.count = 1;
  cloud.fields = {x, y, z, timestamp_field};
  cloud.point_step = 20;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  const float points[][3] = {{1.01F, 0.01F, 0.01F}, {1.02F, 0.02F, 0.02F}, {0.01F, 0.01F, 0.01F},
                             {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F}};
  for (std::size_t i = 0; i < 4; ++i) {
    auto* point = cloud.data.data() + i * cloud.point_step;
    std::memcpy(point, points[i], sizeof(points[i]));
    const double timestamp = 100.0 + static_cast<double>(i);
    std::memcpy(point + 12, &timestamp, sizeof(timestamp));
  }
  return cloud;
}

}  // namespace

TEST(PointCloudOptimizer, FiltersAndPreservesPointFields) {
  zhao_lio_pointcloud_preprocessor::PointCloudOptimizer optimizer({0.1, 10.0, 0.1});
  const auto output = optimizer.optimize(make_cloud());
  ASSERT_EQ(output.width, 1U);
  ASSERT_EQ(output.height, 1U);
  ASSERT_EQ(output.point_step, 20U);
  EXPECT_EQ(output.header.frame_id, "mid70_link");
  double timestamp = 0.0;
  std::memcpy(&timestamp, output.data.data() + 12, sizeof(timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 100.0);
  EXPECT_TRUE(output.is_dense);
}

TEST(PointCloudOptimizer, DisablesVoxelDownsamplingWhenSizeIsNonPositive) {
  zhao_lio_pointcloud_preprocessor::PointCloudOptimizer optimizer({0.1, 10.0, 0.0});
  const auto output = optimizer.optimize(make_cloud());
  EXPECT_EQ(output.width, 2U);
}

TEST(PointCloudOptimizer, RejectsMalformedLayoutsBeforeDynamicFiltering) {
  zhao_lio_pointcloud_preprocessor::PointCloudOptimizer optimizer;
  auto cloud = make_cloud();
  cloud.fields.front().offset = cloud.point_step;
  EXPECT_THROW(optimizer.optimize(cloud), std::invalid_argument);
  cloud = make_cloud();
  cloud.data.pop_back();
  EXPECT_THROW(optimizer.optimize(cloud), std::invalid_argument);
  cloud = make_cloud();
  cloud.is_bigendian = true;
  EXPECT_THROW(optimizer.optimize(cloud), std::invalid_argument);
}

namespace {

sensor_msgs::msg::PointCloud2 threshold_cloud() {
  auto cloud = make_cloud();
  cloud.width = 5;
  cloud.point_step = 24;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.fields.back().offset = 16;
  sensor_msgs::msg::PointField intensity;
  intensity.name = "intensity";
  intensity.offset = 12;
  intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
  intensity.count = 1;
  cloud.fields.push_back(intensity);
  cloud.data.resize(cloud.row_step);
  const float values[][4] = {
      {2.0F, -1.25F, 3.0F, 10.0F}, {2.0F, -1.125F, 3.0F, 5.0F},
      {2.0F, 0.0F, 3.0F, 4.0F}, {2.0F, 0.0F, 3.0F, std::numeric_limits<float>::quiet_NaN()},
      {2.0F, 0.0F, 3.0F, 20.0F}};
  for (std::size_t index = 0; index < cloud.width; ++index) {
    std::memcpy(cloud.data.data() + index * cloud.point_step, values[index], sizeof(values[index]));
    const double timestamp = 100.0 + index;
    std::memcpy(cloud.data.data() + index * cloud.point_step + 16, &timestamp, sizeof(timestamp));
  }
  return cloud;
}

Eigen::Isometry3d height_transform() {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0;
  transform.translation().z() = 1.0;
  return transform;
}

}

TEST(PointCloudOptimizer, HeightUsesFullBaseTransformAndPreservesBoundaryAndRecords) {
  zhao_lio_pointcloud_preprocessor::PointCloudOptimizer optimizer({0.1, 10.0, 0.0, true, -0.125, 5.0});
  zhao_lio_pointcloud_preprocessor::OptimizerStats stats;
  const auto input = threshold_cloud();
  const auto output = optimizer.optimize(input, height_transform(), &stats);
  ASSERT_EQ(output.width, 2U);
  EXPECT_EQ(output.fields, input.fields);
  EXPECT_EQ(output.header, input.header);
  EXPECT_EQ(std::memcmp(output.data.data(), input.data.data() + input.point_step, input.point_step), 0);
  EXPECT_EQ(std::memcmp(output.data.data() + output.point_step, input.data.data() + 4 * input.point_step, input.point_step), 0);
  EXPECT_EQ(stats.low_height, 1U);
  EXPECT_EQ(stats.low_intensity, 2U);
  EXPECT_TRUE(stats.height_applied);
  EXPECT_TRUE(stats.intensity_applied);
}

TEST(PointCloudOptimizer, MissingTransformOrIntensitySkipsOnlyUnavailableFilter) {
  zhao_lio_pointcloud_preprocessor::PointCloudOptimizer optimizer({0.1, 10.0, 0.0, true, -0.125, 5.0});
  zhao_lio_pointcloud_preprocessor::OptimizerStats stats;
  auto input = threshold_cloud();
  EXPECT_EQ(optimizer.optimize(input, std::nullopt, &stats).width, 3U);
  EXPECT_FALSE(stats.height_applied);
  EXPECT_TRUE(stats.intensity_applied);
  input.fields.pop_back();
  EXPECT_EQ(optimizer.optimize(input, height_transform(), &stats).width, 4U);
  EXPECT_TRUE(stats.height_applied);
  EXPECT_FALSE(stats.intensity_applied);
}

TEST(PointCloudOptimizer, IntensityFilterPrecedesVoxelSelectionAndSupportsIntegers) {
  zhao_lio_pointcloud_preprocessor::PointCloudOptimizer optimizer({0.1, 10.0, 10.0, false, -0.125, 5.0});
  auto input = threshold_cloud();
  input.fields.back().datatype = sensor_msgs::msg::PointField::UINT8;
  for (std::size_t index = 0; index < input.width; ++index) {
    input.data[index * input.point_step + 12] = index == 4 ? 5 : 4;
  }
  const auto output = optimizer.optimize(input);
  ASSERT_EQ(output.width, 1U);
  EXPECT_EQ(std::memcmp(output.data.data(), input.data.data() + 4 * input.point_step, input.point_step), 0);
}

TEST(PointCloudOptimizer, ThresholdFiltersSupportPaddedRowsAndEmptyOutput) {
  zhao_lio_pointcloud_preprocessor::PointCloudOptimizer optimizer({0.1, 10.0, 0.0, true, -0.125, 5.0});
  auto input = threshold_cloud();
  const auto flat = input.data;
  input.height = input.width;
  input.width = 1;
  input.row_step = input.point_step + 8;
  input.data.resize(input.height * input.row_step);
  for (std::size_t row = 0; row < input.height; ++row) {
    std::memcpy(input.data.data() + row * input.row_step, flat.data() + row * input.point_step, input.point_step);
  }
  EXPECT_EQ(optimizer.optimize(input, height_transform()).width, 2U);
  zhao_lio_pointcloud_preprocessor::PointCloudOptimizer strict({0.1, 10.0, 0.0, false, 0.0, 200.0});
  const auto empty = strict.optimize(input);
  EXPECT_EQ(empty.width, 0U);
  EXPECT_EQ(empty.row_step, 0U);
  EXPECT_TRUE(empty.data.empty());
  EXPECT_EQ(empty.header, input.header);
}

TEST(PointCloudOptimizer, RejectsInvalidIntensityAndThresholds) {
  zhao_lio_pointcloud_preprocessor::PointCloudOptimizer optimizer({0.1, 10.0, 0.0, true, -0.125, 5.0});
  auto input = threshold_cloud();
  input.fields.back().offset = input.point_step;
  EXPECT_THROW(optimizer.optimize(input), std::invalid_argument);
  input = threshold_cloud();
  input.fields.back().count = 2;
  EXPECT_THROW(optimizer.optimize(input), std::invalid_argument);
  using zhao_lio_pointcloud_preprocessor::PointCloudOptimizer;
  EXPECT_THROW(PointCloudOptimizer invalid({0.1, 10.0, 0.0, true, 0.0, -1.0}), std::invalid_argument);
  EXPECT_THROW(PointCloudOptimizer invalid({0.1, 10.0, 0.0, true,
      std::numeric_limits<double>::quiet_NaN(), 5.0}), std::invalid_argument);
}

TEST(PointCloudOptimizer, UpperHeightUsesBaseFrameAndKeepsFourMeterBoundary) {
  zhao_lio_pointcloud_preprocessor::PointCloudOptimizer optimizer({0.1, 10.0, 0.0, true, -0.125, 5.0, 4.0});
  auto input = threshold_cloud();
  const float at_limit = 3.0F;
  const float above_limit = 3.25F;
  std::memcpy(input.data.data() + 4, &at_limit, sizeof(at_limit));
  std::memcpy(input.data.data() + input.point_step + 4, &above_limit, sizeof(above_limit));
  zhao_lio_pointcloud_preprocessor::OptimizerStats stats;
  const auto output = optimizer.optimize(input, height_transform(), &stats);
  ASSERT_EQ(output.width, 2U);
  EXPECT_EQ(stats.high_height, 1U);
  EXPECT_EQ(stats.low_height, 0U);
  EXPECT_EQ(std::memcmp(output.data.data(), input.data.data(), input.point_step), 0);
  EXPECT_EQ(std::memcmp(output.data.data() + output.point_step,
                        input.data.data() + 4 * input.point_step, input.point_step), 0);
  using zhao_lio_pointcloud_preprocessor::PointCloudOptimizer;
  EXPECT_THROW(PointCloudOptimizer invalid({0.1, 10.0, 0.0, true, 4.0, 5.0, 4.0}), std::invalid_argument);
}
