#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "zhao_lio_pointcloud_preprocessor/cluster_tracker.hpp"

namespace {

using zhao_lio_pointcloud_preprocessor::ClusterTracker;
using zhao_lio_pointcloud_preprocessor::ClusterConfig;
using Vector = Eigen::Vector3d;

std::vector<Vector> background() {
  std::vector<Vector> points;
  for (int horizontal = -20; horizontal <= 20; ++horizontal) {
    for (int vertical = 1; vertical <= 15; ++vertical) {
      points.emplace_back(8.0, horizontal * 0.15, vertical * 0.15);
    }
    for (int depth = 5; depth <= 40; ++depth) {
      points.emplace_back(depth * 0.15, horizontal * 0.15, 0.0);
    }
  }
  return points;
}

std::vector<Vector> person(const Vector& center) {
  std::vector<Vector> points;
  for (int horizontal = -2; horizontal <= 2; ++horizontal) {
    for (int vertical = 0; vertical <= 16; ++vertical) {
      points.push_back(center + Vector(0.03 * std::cos(vertical), horizontal * 0.09, 0.16 + vertical * 0.10));
    }
    points.push_back(center + Vector(0.0, horizontal * 0.09, 0.06));
  }
  return points;
}

void append(std::vector<Vector>& destination, const std::vector<Vector>& source) {
  destination.insert(destination.end(), source.begin(), source.end());
}

Eigen::Isometry3d pose(int frame) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(frame * 0.04, Vector::UnitZ()).toRotationMatrix();
  transform.translation() = Vector(frame * 0.03, frame * -0.02, 0.0);
  return transform;
}

}

TEST(ClusterTracker, LateralMotionRemovesWholePersonIncludingOverlappingPoints) {
  ClusterTracker tracker;
  const auto scenery = background();
  const auto body_size = person(Vector(3.0, 0.0, 0.0)).size();
  std::uint32_t identifier = 0;
  for (int frame = 0; frame < 10; ++frame) {
    auto points = scenery;
    append(points, person(Vector(3.0, frame * 0.07, 0.0)));
    const auto result = tracker.update(points, 10.0 + frame * 0.1,
        Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    ASSERT_EQ(result.clusters.size(), 1U);
    if (frame == 0) identifier = result.clusters.front().id;
    EXPECT_EQ(result.clusters.front().id, identifier);
    if (frame < 4) {
      EXPECT_EQ(std::count(result.remove.begin(), result.remove.end(), true), 0);
    } else {
      EXPECT_TRUE(result.clusters.front().moving);
      EXPECT_NEAR(result.clusters.front().speed, 0.7, 0.05);
      EXPECT_EQ(std::count(result.remove.begin(), result.remove.end(), true), body_size);
      for (std::size_t index = 0; index < points.size(); ++index) {
        EXPECT_EQ(result.remove[index], index >= scenery.size());
      }
    }
  }
}

TEST(ClusterTracker, EgoMotionAndStaticResamplingDoNotRemoveStationaryPerson) {
  ClusterTracker tracker;
  Eigen::Isometry3d previous = Eigen::Isometry3d::Identity();
  for (int frame = 0; frame < 10; ++frame) {
    auto points = background();
    auto body = person(Vector(3.0, 0.0, 0.0));
    for (std::size_t index = 0; index < body.size(); ++index) {
      body[index].x() += 0.015 * std::sin(index + frame * 1.1);
      body[index].y() += 0.015 * std::cos(index * 1.3 + frame);
    }
    append(points, body);
    const auto current = pose(frame);
    for (auto& point : points) point = current.inverse() * point;
    const auto result = tracker.update(points, 10.0 + frame * 0.1, previous.inverse() * current,
                                       Eigen::Isometry3d::Identity());
    EXPECT_EQ(std::count(result.remove.begin(), result.remove.end(), true), 0);
    previous = current;
  }
}

TEST(ClusterTracker, TwoMovingClustersAreTrackedSeparatelyFromStaticCluster) {
  ClusterTracker tracker;
  Eigen::Isometry3d previous = Eigen::Isometry3d::Identity();
  const auto scenery = background();
  const auto body_size = person(Vector(3.0, 0.0, 0.0)).size();
  for (int frame = 0; frame < 9; ++frame) {
    auto points = scenery;
    append(points, person(Vector(3.0 + frame * 0.07, -1.0, 0.0)));
    append(points, person(Vector(4.0, 1.0 + frame * 0.07, 0.0)));
    append(points, person(Vector(2.0, 2.5, 0.0)));
    const auto current = pose(frame);
    for (auto& point : points) point = current.inverse() * point;
    const auto result = tracker.update(points, 10.0 + frame * 0.1, previous.inverse() * current,
                                       Eigen::Isometry3d::Identity());
    if (frame >= 4) {
      ASSERT_EQ(result.clusters.size(), 3U);
      EXPECT_EQ(std::count(result.remove.begin(), result.remove.end(), true), 2 * body_size);
      for (std::size_t index = 0; index < points.size(); ++index) {
        EXPECT_EQ(result.remove[index], index >= scenery.size() && index < scenery.size() + 2 * body_size);
      }
    }
    previous = current;
  }
}

TEST(ClusterTracker, NewlyVisibleStaticObjectAndStoppedObjectAreKept) {
  ClusterTracker tracker;
  for (int frame = 0; frame < 18; ++frame) {
    auto points = background();
    if (frame >= 3) append(points, person(Vector(3.0, std::min(frame, 10) * 0.07, 0.0)));
    const auto result = tracker.update(points, 10.0 + frame * 0.1,
        Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    if (frame < 7 || frame >= 14) EXPECT_EQ(std::count(result.remove.begin(), result.remove.end(), true), 0);
    if (frame >= 7 && frame <= 10) EXPECT_GT(std::count(result.remove.begin(), result.remove.end(), true), 0);
  }
}

TEST(ClusterTracker, TimeReversalAndLongGapRestartConfirmation) {
  ClusterTracker tracker;
  auto points = background();
  append(points, person(Vector(3.0, 0.0, 0.0)));
  tracker.update(points, 10.0, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
  for (double time : {9.0, 12.0}) {
    const auto result = tracker.update(points, time, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    ASSERT_EQ(result.clusters.size(), 1U);
    EXPECT_EQ(result.clusters.front().state, "warming");
    EXPECT_EQ(std::count(result.remove.begin(), result.remove.end(), true), 0);
  }
}

TEST(ClusterTracker, GroundHeightUsesBaseTransformAndLargeWallsAreNotObjects) {
  ClusterTracker tracker;
  const auto result = tracker.update(background(), 10.0,
      Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
  EXPECT_TRUE(result.clusters.empty());
  EXPECT_GT(result.rejected_components, 0U);
  auto points = person(Vector(3.0, 0.0, 0.0));
  Eigen::Isometry3d lidar_to_base = Eigen::Isometry3d::Identity();
  lidar_to_base.translation().z() = 1.2;
  for (auto& point : points) point.z() -= 1.2;
  tracker.reset();
  const auto translated = tracker.update(points, 10.0, Eigen::Isometry3d::Identity(), lidar_to_base);
  ASSERT_EQ(translated.clusters.size(), 1U);
  EXPECT_EQ(translated.clusters.front().points, points.size());
}
