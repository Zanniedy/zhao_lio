#include <cmath>
#include <cstring>
#include <limits>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_field.hpp>

#include "zhao_lio_pointcloud_preprocessor/dynamic_filter.hpp"

namespace {

using zhao_lio_pointcloud_preprocessor::DynamicConfig;
using zhao_lio_pointcloud_preprocessor::DynamicFilter;
using Cloud = sensor_msgs::msg::PointCloud2;
using Vector = Eigen::Vector3d;

std::vector<Vector> scene(double object_x = 0.0) {
  std::vector<Vector> points;
  for (int horizontal = -25; horizontal <= 25; ++horizontal) {
    for (int vertical = -15; vertical <= 15; ++vertical) {
      points.emplace_back(8.0 + 0.15 * std::sin(horizontal * 0.7), horizontal * 0.12, vertical * 0.12);
    }
  }
  if (object_x > 0.0) {
    for (int horizontal = -4; horizontal <= 4; ++horizontal) {
      for (int vertical = -4; vertical <= 4; ++vertical) {
        points.emplace_back(object_x, horizontal * 0.06, vertical * 0.06);
      }
    }
  }
  return points;
}

Cloud cloud(double time, const std::vector<Vector>& points, double duration = 0.0,
            const Vector& gyro = Vector::Zero(), const Vector& translation = Vector::Zero()) {
  Cloud output;
  output.header.frame_id = "mid70_link";
  const auto nanoseconds = static_cast<std::int64_t>(std::llround(time * 1e9));
  output.header.stamp.sec = nanoseconds / 1000000000;
  output.header.stamp.nanosec = nanoseconds % 1000000000;
  output.height = 1;
  output.width = points.size();
  output.point_step = 24;
  output.row_step = output.width * output.point_step;
  output.is_dense = true;
  const std::vector<std::string> names{"x", "y", "z", "intensity", "timestamp"};
  for (std::size_t index = 0; index < names.size(); ++index) {
    sensor_msgs::msg::PointField field;
    field.name = names[index];
    field.offset = index * 4;
    field.count = 1;
    field.datatype = index == 4 ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
    output.fields.push_back(field);
  }
  output.data.resize(output.row_step);
  for (std::size_t index = 0; index < points.size(); ++index) {
    const double point_time = time + duration * index / std::max<std::size_t>(1, points.size() - 1);
    const Eigen::Quaterniond orientation = gyro.norm() == 0.0 ? Eigen::Quaterniond::Identity() :
        Eigen::Quaterniond(Eigen::AngleAxisd(gyro.norm() * (point_time - 10.0), gyro.normalized()));
    const Vector local = orientation.conjugate() * (points[index] - translation);
    auto* data = output.data.data() + index * output.point_step;
    for (int axis = 0; axis < 3; ++axis) {
      const float value = local[axis];
      std::memcpy(data + axis * 4, &value, sizeof(value));
    }
    const float intensity = index;
    std::memcpy(data + 12, &intensity, sizeof(intensity));
    std::memcpy(data + 16, &point_time, sizeof(point_time));
  }
  return output;
}

void imu(DynamicFilter& filter, const Vector& gyro = Vector::Zero()) {
  for (int sample = 0; sample <= 300; ++sample) filter.add_imu(9.9 + sample * 0.005, gyro);
}

void preserves_partition(const Cloud& input, const Cloud& kept, const Cloud& removed) {
  ASSERT_EQ(kept.width + removed.width, input.width);
  EXPECT_EQ(kept.header, input.header);
  EXPECT_EQ(removed.header, input.header);
  EXPECT_EQ(kept.fields, input.fields);
  EXPECT_EQ(removed.fields, input.fields);
  for (const auto* output : {&kept, &removed}) {
    EXPECT_EQ(output->row_step, output->width * output->point_step);
    for (std::size_t index = 0; index < output->width; ++index) {
      const auto* data = output->data.data() + index * output->point_step;
      float identity;
      std::memcpy(&identity, data + 12, sizeof(identity));
      ASSERT_LT(static_cast<std::size_t>(identity), input.width);
      EXPECT_EQ(std::memcmp(data, input.data.data() + static_cast<std::size_t>(identity) * input.point_step,
                            input.point_step), 0);
    }
  }
}

}

TEST(DynamicFilter, StaticSceneWithRotationTranslationAndPerPointTimesIsKept) {
  DynamicFilter filter;
  const Vector gyro(0.12, -0.08, 0.35);
  imu(filter, gyro);
  for (int frame = 0; frame < 5; ++frame) {
    const auto input = cloud(10.0 + frame * 0.1, scene(), 0.08, gyro, Vector(frame * 0.04, 0.0, 0.0));
    const auto result = filter.filter(input);
    EXPECT_EQ(result.kept.data, input.data);
    EXPECT_EQ(result.removed.width, 0U);
    EXPECT_EQ(result.reason, frame == 0 ? "warmup" : "matched");
  }
}

TEST(DynamicFilter, ClusterModeRemovesWholeMovingBodyAndPreservesOriginalRecords) {
  DynamicConfig config;
  config.cluster_tracking_enabled = true;
  DynamicFilter filter(config);
  imu(filter);
  const auto background = scene();
  for (int frame = 0; frame < 9; ++frame) {
    auto points = background;
    for (int horizontal = -2; horizontal <= 2; ++horizontal) {
      for (int vertical = 0; vertical <= 16; ++vertical) {
        points.emplace_back(3.0, horizontal * 0.09 + frame * 0.07, 0.16 + vertical * 0.10);
      }
      points.emplace_back(3.0, horizontal * 0.09 + frame * 0.07, 0.06);
    }
    const auto input = cloud(10.0 + frame * 0.1, points);
    const auto result = filter.filter(input, Eigen::Isometry3d::Identity());
    ASSERT_EQ(result.reason, frame == 0 ? "warmup" : "matched");
    preserves_partition(input, result.kept, result.removed);
    if (frame < 4) {
      EXPECT_EQ(result.removed.width, 0U);
    } else {
      EXPECT_EQ(result.removed.width, 90U);
      EXPECT_EQ(result.kept.width, background.size());
      EXPECT_EQ(std::memcmp(result.kept.data.data(), input.data.data(), result.kept.data.size()), 0);
      ASSERT_EQ(result.clusters.size(), 1U);
      EXPECT_TRUE(result.clusters.front().removed);
    }
  }
}

TEST(DynamicFilter, ClusterModeRequiresBaseTransform) {
  DynamicConfig config;
  config.cluster_tracking_enabled = true;
  DynamicFilter filter(config);
  imu(filter);
  const auto input = cloud(10.0, scene(3.0));
  const auto result = filter.filter(input);
  EXPECT_EQ(result.reason, "cluster_tf_unavailable");
  EXPECT_EQ(result.kept.data, input.data);
  EXPECT_EQ(result.removed.width, 0U);
}

TEST(DynamicFilter, MovingClusterRequiresTwoConfirmationsAndPreservesBytes) {
  DynamicFilter filter;
  imu(filter);
  EXPECT_EQ(filter.filter(cloud(10.0, scene(3.8))).reason, "warmup");
  const auto candidate = filter.filter(cloud(10.1, scene(3.35)));
  EXPECT_EQ(candidate.reason, "matched");
  EXPECT_GT(candidate.candidates, 0U);
  EXPECT_EQ(candidate.removed.width, 0U);
  const auto input = cloud(10.2, scene(2.9));
  const auto result = filter.filter(input);
  EXPECT_EQ(result.reason, "matched");
  EXPECT_GT(result.removed.width, 0U);
  EXPECT_LE(result.removed.width, 81U);
  preserves_partition(input, result.kept, result.removed);
  for (std::size_t index = 0; index < result.removed.width; ++index) {
    float object_x;
    std::memcpy(&object_x, result.removed.data.data() + index * result.removed.point_step, sizeof(object_x));
    EXPECT_NEAR(object_x, 2.9, 1e-5);
  }
}

TEST(DynamicFilter, NewlyExposedBackgroundIsNotDynamic) {
  DynamicFilter filter;
  imu(filter);
  filter.filter(cloud(10.0, scene(3.8)));
  for (int frame = 1; frame <= 3; ++frame) {
    const auto input = cloud(10.0 + frame * 0.1, scene());
    const auto result = filter.filter(input);
    EXPECT_EQ(result.reason, "matched");
    EXPECT_EQ(result.kept.data, input.data);
    EXPECT_EQ(result.removed.width, 0U);
  }
}

TEST(DynamicFilter, MissingImuAndLongGapsPassThrough) {
  DynamicFilter filter;
  const auto input = cloud(10.0, scene());
  EXPECT_EQ(filter.filter(input).reason, "imu_not_bracketed");
  imu(filter);
  EXPECT_EQ(filter.filter(input).reason, "warmup");
  const auto late = cloud(10.8, scene(3.0));
  EXPECT_EQ(filter.filter(late).reason, "dt_too_long");
  filter.add_imu(12.0, Vector::Zero());
  const auto gap = filter.filter(cloud(12.0, scene()));
  EXPECT_EQ(gap.reason, "warmup");
  EXPECT_EQ(gap.removed.width, 0U);
}

TEST(DynamicFilter, TimeReversalResetsHistory) {
  DynamicFilter filter;
  imu(filter);
  filter.filter(cloud(10.2, scene()));
  EXPECT_EQ(filter.filter(cloud(10.1, scene())).reason, "time_reversed");
  EXPECT_FALSE(filter.has_imu_until(10.1));
  imu(filter);
  EXPECT_EQ(filter.filter(cloud(10.1, scene())).reason, "warmup");
  filter.add_imu(5.0, Vector::Zero());
  EXPECT_FALSE(filter.has_imu_until(10.1));
}

TEST(DynamicFilter, UnreliableRegistrationAndRemovalLimitKeepEverything) {
  DynamicConfig config;
  config.max_removed_ratio = 0.001;
  DynamicFilter filter(config);
  imu(filter);
  filter.filter(cloud(10.0, scene(3.8)));
  filter.filter(cloud(10.1, scene(3.35)));
  const auto input = cloud(10.2, scene(2.9));
  const auto result = filter.filter(input);
  EXPECT_EQ(result.reason, "removal_limit");
  EXPECT_EQ(result.kept.data, input.data);
  EXPECT_EQ(result.removed.width, 0U);
  auto displaced = scene();
  for (auto& point : displaced) point.x() += 5.0;
  const auto failed = filter.filter(cloud(10.3, displaced));
  EXPECT_EQ(failed.reason, "registration_unreliable");
  EXPECT_EQ(failed.removed.width, 0U);
}

TEST(DynamicFilter, ValidatesTimingLayoutAndParameters) {
  DynamicFilter filter;
  auto input = cloud(10.0, scene());
  EXPECT_DOUBLE_EQ(filter.scan_end(input), 10.0);
  input.fields.back().offset = input.point_step;
  EXPECT_THROW(filter.scan_end(input), std::invalid_argument);
  input = cloud(10.0, scene(), 0.5);
  EXPECT_THROW(filter.scan_end(input), std::invalid_argument);
  input = cloud(10.0, scene());
  input.data.pop_back();
  EXPECT_THROW(filter.scan_end(input), std::invalid_argument);
  input = cloud(10.0, scene());
  input.is_bigendian = true;
  EXPECT_THROW(filter.scan_end(input), std::invalid_argument);
  input = cloud(10.0, scene());
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  std::memcpy(input.data.data() + 16, &invalid, sizeof(invalid));
  EXPECT_THROW(filter.scan_end(input), std::invalid_argument);
  DynamicConfig config;
  config.min_dt = -1.0;
  EXPECT_THROW(DynamicFilter invalid_filter(config), std::invalid_argument);
}

TEST(DynamicFilter, SupportsNanosecondTimesAndOrganizedRows) {
  DynamicFilter filter;
  auto input = cloud(1700000000.0, scene());
  for (std::size_t index = 0; index < input.width; ++index) {
    const double nanoseconds = 1700000000.0 * 1e9;
    std::memcpy(input.data.data() + index * input.point_step + 16, &nanoseconds, sizeof(nanoseconds));
  }
  EXPECT_DOUBLE_EQ(filter.scan_end(input), 1700000000.0);
  input = cloud(10.0, {Vector(1.0, 2.0, 3.0), Vector(4.0, 5.0, 6.0)});
  input.width = 1;
  input.height = 2;
  input.row_step = input.point_step + 8;
  input.data.insert(input.data.begin() + input.point_step, 8, 0);
  input.data.resize(input.row_step * 2);
  EXPECT_DOUBLE_EQ(filter.scan_end(input), 10.0);
}

TEST(DynamicFilter, DisabledFilterIsByteIdentical) {
  DynamicConfig config;
  config.enabled = false;
  DynamicFilter filter(config);
  const auto input = cloud(10.0, scene());
  const auto result = filter.filter(input);
  EXPECT_EQ(result.reason, "disabled");
  EXPECT_EQ(result.kept, input);
  EXPECT_EQ(result.removed.width, 0U);
}

TEST(DynamicFilter, IncreasedSensitivityDetectsSmallerMotionWithoutSingleFrameDeletion) {
  DynamicFilter filter;
  imu(filter);
  filter.filter(cloud(10.0, scene(3.8)));
  const auto first = filter.filter(cloud(10.1, scene(3.6)));
  EXPECT_EQ(first.reason, "matched");
  EXPECT_GT(first.clustered, 0U);
  EXPECT_EQ(first.removed.width, 0U);
  const auto second = filter.filter(cloud(10.2, scene(3.4)));
  EXPECT_EQ(second.reason, "matched");
  EXPECT_GT(second.removed.width, 0U);
  EXPECT_EQ(second.confirmed, second.removed.width);
  DynamicConfig conservative;
  conservative.static_distance = 0.18;
  conservative.free_space_margin = 0.40;
  conservative.min_cluster_points = 5;
  conservative.min_visibility_bins = 5;
  DynamicFilter old_filter(conservative);
  imu(old_filter);
  old_filter.filter(cloud(10.0, scene(3.8)));
  old_filter.filter(cloud(10.1, scene(3.6)));
  EXPECT_EQ(old_filter.filter(cloud(10.2, scene(3.4))).removed.width, 0U);
}

TEST(DynamicFilter, StaticResamplingIsNotMotion) {
  DynamicFilter filter;
  imu(filter);
  for (int frame = 0; frame < 5; ++frame) {
    auto points = scene();
    for (std::size_t index = 0; index < points.size(); ++index) {
      points[index].x() = 8.0;
      points[index].y() += 0.05 * std::sin(index * 1.7 + frame);
      points[index].z() += 0.05 * std::cos(index * 1.3 + frame);
    }
    const auto input = cloud(10.0 + frame * 0.1, points);
    const auto result = filter.filter(input);
    EXPECT_EQ(result.reason, frame == 0 ? "warmup" : "matched");
    EXPECT_EQ(result.kept.data, input.data);
    EXPECT_EQ(result.removed.width, 0U);
  }
}
