#include <algorithm>
#include <gtest/gtest.h>

#include "dynablox_ros2/adapter.hpp"

TEST(OfficialEverFree, RequiresObservedNeighborhoodAndBurnIn) {
  auto layer = std::make_shared<dynablox::TsdfLayer>(0.2, 16);
  auto block = layer->allocateBlockPtrByIndex(voxblox::BlockIndex::Zero());
  for (std::size_t index = 0; index < block->num_voxels(); ++index) {
    auto& voxel = block->getVoxelByLinearIndex(index);
    voxel.weight = 1.0;
    voxel.distance = 1.0;
  }
  dynablox::EverFreeIntegrator::Config config;
  config.num_threads = 1;
  config.burn_in_period = 5;
  config.neighbor_connectivity = 26;
  dynablox::EverFreeIntegrator integrator(config, layer);
  block->updated().set();
  integrator.updateEverFreeVoxels(3);
  EXPECT_FALSE(block->getVoxelByVoxelIndex({8, 8, 8}).ever_free);
  block->updated().set();
  integrator.updateEverFreeVoxels(6);
  EXPECT_TRUE(block->getVoxelByVoxelIndex({8, 8, 8}).ever_free);
  EXPECT_FALSE(block->getVoxelByVoxelIndex({0, 0, 0}).ever_free);
}

TEST(OfficialEverFree, PersistentOccupationRevokesEverFree) {
  auto layer = std::make_shared<dynablox::TsdfLayer>(0.2, 16);
  auto block = layer->allocateBlockPtrByIndex(voxblox::BlockIndex::Zero());
  auto& target = block->getVoxelByVoxelIndex({8, 8, 8});
  target.weight = 1.0;
  target.distance = 0.0;
  target.ever_free = true;
  dynablox::EverFreeIntegrator::Config config;
  config.num_threads = 1;
  config.counter_to_reset = 3;
  dynablox::EverFreeIntegrator integrator(config, layer);
  for (int frame = 1; frame <= 3; ++frame) {
    block->updated().set();
    integrator.updateEverFreeVoxels(frame);
  }
  EXPECT_FALSE(target.ever_free);
  EXPECT_GE(target.occ_counter, 3);
}

TEST(OfficialClustering, GrowsFromEverFreeIntoOccupiedNeighbor) {
  auto layer = std::make_shared<dynablox::TsdfLayer>(0.2, 16);
  const voxblox::BlockIndex block_index = voxblox::BlockIndex::Zero();
  auto block = layer->allocateBlockPtrByIndex(block_index);
  dynablox::Cloud cloud;
  dynablox::CloudInfo info;
  dynablox::BlockToPointMap point_map;
  const std::vector<voxblox::VoxelIndex> indices{{8, 8, 8}, {9, 8, 8}};
  for (std::size_t index = 0; index < indices.size(); ++index) {
    auto& voxel = block->getVoxelByVoxelIndex(indices[index]);
    voxel.last_lidar_occupied = 10;
    voxel.ever_free = index == 0;
    const auto position = block->computeCoordinatesFromVoxelIndex(indices[index]);
    cloud.push_back(dynablox::Point(position.x(), position.y(), position.z()));
    info.points.emplace_back();
    point_map[block_index][indices[index]].push_back(index);
  }
  dynablox::Clustering::Config config;
  config.min_cluster_size = 1;
  config.max_cluster_size = 100;
  config.min_extent = 0.0;
  config.max_extent = 100.0;
  config.grow_clusters_twice = false;
  dynablox::Clustering clustering(config, layer);
  auto clusters = clustering.performClustering(point_map, {{block_index, indices.front()}}, 10, cloud, info);
  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_EQ(clusters.front().points.size(), 2U);
  EXPECT_TRUE(info.points[0].cluster_level_dynamic);
  EXPECT_TRUE(info.points[1].cluster_level_dynamic);
  dynablox::Tracking::Config tracking_config;
  dynablox::Tracking tracking(tracking_config);
  tracking.track(cloud, clusters, info);
  EXPECT_TRUE(info.points[1].object_level_dynamic);
}

TEST(DynabloxAdapter, StaticScanBuildsTsdfAndTimeReversalClearsHistory) {
  dynablox_ros2::DynabloxConfig config;
  config.num_threads = 1;
  dynablox_ros2::DynabloxAdapter adapter(config);
  std::vector<Eigen::Vector3d> points;
  for (int lateral = -20; lateral <= 20; ++lateral) {
    for (int height = -15; height <= 15; ++height) points.emplace_back(4.05, lateral * 0.1, height * 0.1);
  }
  for (int frame = 0; frame < 10; ++frame) {
    const auto result = adapter.update(points, Eigen::Isometry3d::Identity(), 10.0 + frame * 0.1);
    EXPECT_EQ(std::count(result.labels.begin(), result.labels.end(), dynablox_ros2::PointLabel::Dynamic), 0);
  }
  EXPECT_GT(adapter.static_map().size(), 100U);
  EXPECT_GT(adapter.free_centers(10000).size(), 100U);
  const auto result = adapter.update(points, Eigen::Isometry3d::Identity(), 5.0);
  EXPECT_EQ(result.reason, "time_reset");
  EXPECT_EQ(result.frame, 1);
  EXPECT_TRUE(adapter.free_centers(10000).empty());
}

TEST(DynabloxAdapter, InvalidDataAndParametersRejected) {
  dynablox_ros2::DynabloxConfig config;
  config.num_threads = 0;
  EXPECT_THROW(dynablox_ros2::DynabloxAdapter{config}, std::invalid_argument);
  config.num_threads = 1;
  dynablox_ros2::DynabloxAdapter adapter(config);
  EXPECT_THROW(adapter.update({}, Eigen::Isometry3d::Identity(), -1.0), std::invalid_argument);
  auto transform = Eigen::Isometry3d::Identity();
  transform.linear()(0, 0) = 2.0;
  EXPECT_THROW(adapter.update({}, transform, 10.0), std::invalid_argument);
}

TEST(DynabloxAdapter, DetectsNewObjectInsidePreviouslyFreeVolume) {
  dynablox_ros2::DynabloxConfig config;
  config.num_threads = 1;
  config.clustering.min_cluster_size = 20;
  config.clustering.min_extent = 0.0;
  dynablox_ros2::DynabloxAdapter adapter(config);
  std::vector<Eigen::Vector3d> points;
  for (int lateral = -30; lateral <= 30; ++lateral) {
    for (int height = -25; height <= 25; ++height) points.emplace_back(4.05, lateral * 0.1, height * 0.1);
  }
  for (int frame = 0; frame < 12; ++frame) adapter.update(points, Eigen::Isometry3d::Identity(), 10.0 + frame * 0.1);
  const auto wall_points = points.size();
  for (int lateral = -3; lateral <= 3; ++lateral) {
    for (int height = -8; height <= 8; ++height) points.emplace_back(2.05, lateral * 0.1, height * 0.1);
  }
  const auto result = adapter.update(points, Eigen::Isometry3d::Identity(), 11.2);
  EXPECT_GT(result.seeds.size(), 10U);
  EXPECT_GT(std::count(result.labels.begin(), result.labels.end(), dynablox_ros2::PointLabel::Dynamic), 20);
  for (std::size_t index = 0; index < wall_points; ++index) EXPECT_NE(result.labels[index], dynablox_ros2::PointLabel::Dynamic);
}
