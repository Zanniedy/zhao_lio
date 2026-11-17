#include "dynablox_ros2/adapter.hpp"

#include <cmath>
#include <stdexcept>

namespace dynablox_ros2 {

DynabloxAdapter::DynabloxAdapter(DynabloxConfig config) : config_(std::move(config)) {
  if (!std::isfinite(config_.resolution) || config_.resolution < 0.05 || config_.resolution > 1.0 ||
      !std::isfinite(config_.min_range) || config_.min_range < 0.0 ||
      !std::isfinite(config_.max_range) || config_.max_range <= config_.min_range || config_.max_range > 100.0 ||
      !std::isfinite(config_.map_radius) || config_.map_radius < config_.max_range ||
      !std::isfinite(config_.max_time_gap) || config_.max_time_gap <= 0.0 ||
      (config_.voxels_per_side != 8 && config_.voxels_per_side != 16) || config_.max_blocks < 1 ||
      config_.num_threads < 1 || config_.num_threads > 32 ||
      (config_.integration_method != "merged" && config_.integration_method != "simple" && config_.integration_method != "fast") ||
      config_.ever_free.counter_to_reset < 1 || config_.ever_free.burn_in_period < 1 ||
      !std::isfinite(config_.ever_free.tsdf_occupancy_threshold) || config_.ever_free.tsdf_occupancy_threshold <= 0 ||
      config_.clustering.min_cluster_size < 1 || config_.tracking.min_track_duration < 0 ||
      !std::isfinite(config_.tracking.max_tracking_distance) || config_.tracking.max_tracking_distance <= 0) {
    throw std::invalid_argument("invalid Dynablox adapter config");
  }
  config_.ever_free.num_threads = config_.num_threads;
  reset();
}

void DynabloxAdapter::reset() {
  integrator_.reset();
  ever_free_.reset();
  clustering_.reset();
  tracking_.reset();
  layer_ = std::make_shared<dynablox::TsdfLayer>(config_.resolution, config_.voxels_per_side);
  ever_free_ = std::make_unique<dynablox::EverFreeIntegrator>(config_.ever_free, layer_);
  clustering_ = std::make_unique<dynablox::Clustering>(config_.clustering, layer_);
  tracking_ = std::make_unique<dynablox::Tracking>(config_.tracking);
  voxblox::TsdfIntegratorBase::Config integration;
  integration.default_truncation_distance = 2.0 * config_.resolution;
  integration.max_weight = 1000.0;
  integration.voxel_carving_enabled = true;
  integration.allow_clear = true;
  integration.min_ray_length_m = config_.min_range;
  integration.max_ray_length_m = config_.max_range;
  integration.use_const_weight = true;
  integration.integrator_threads = config_.num_threads;
  integrator_ = voxblox::TsdfIntegratorFactory::create(config_.integration_method, integration, layer_.get());
  frame_ = 0;
  last_time_ = -1.0;
}

DetectionResult DynabloxAdapter::update(const std::vector<Eigen::Vector3d>& world_points,
                                       const Eigen::Isometry3d& world_from_sensor, double time) {
  if (!std::isfinite(time) || time < 0.0 || !world_from_sensor.matrix().allFinite() ||
      (world_from_sensor.linear().transpose() * world_from_sensor.linear() - Eigen::Matrix3d::Identity()).norm() > 1e-4 ||
      std::abs(world_from_sensor.linear().determinant() - 1.0) > 1e-4) {
    throw std::invalid_argument("invalid scan pose/time");
  }
  for (const auto& point : world_points) {
    if (!point.allFinite() || point.cwiseAbs().maxCoeff() > 1e6) throw std::invalid_argument("invalid world point");
  }
  DetectionResult result;
  result.labels.assign(world_points.size(), PointLabel::Unobserved);
  if (last_time_ >= 0.0 && (time <= last_time_ || time - last_time_ > config_.max_time_gap)) {
    reset();
    result.reason = "time_reset";
  }
  if (layer_->getNumberOfAllocatedBlocks() > static_cast<std::size_t>(config_.max_blocks)) {
    reset();
    result.reason = "capacity_reset";
  }
  layer_->removeDistantBlocks(world_from_sensor.translation().cast<float>(), config_.map_radius);
  last_time_ = time;
  ++frame_;
  dynablox::Cloud cloud;
  dynablox::CloudInfo info;
  info.timestamp = static_cast<std::uint64_t>(time * 1e9);
  info.sensor_position = dynablox::Point(world_from_sensor.translation().x(), world_from_sensor.translation().y(), world_from_sensor.translation().z());
  voxblox::Pointcloud local_points;
  std::vector<std::size_t> original_indices;
  const Eigen::Isometry3d sensor_from_world = world_from_sensor.inverse();
  for (std::size_t index = 0; index < world_points.size(); ++index) {
    const Eigen::Vector3d local = sensor_from_world * world_points[index];
    const double range = local.norm();
    cloud.push_back(dynablox::Point(world_points[index].x(), world_points[index].y(), world_points[index].z()));
    local_points.push_back(local.cast<float>());
    dynablox::PointInfo point_info;
    point_info.distance_to_sensor = range;
    info.points.push_back(point_info);
    original_indices.push_back(index);
    if (range >= config_.min_range && range <= config_.max_range) result.labels[index] = PointLabel::Static;
  }

  dynablox::BlockToPointMap point_map;
  std::vector<voxblox::VoxelKey> seeds;
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    const voxblox::Point point = cloud[index].getVector3fMap();
    const auto block_index = layer_->computeBlockIndexFromCoordinates(point);
    auto block = layer_->getBlockPtrByIndex(block_index);
    if (!block) continue;
    const auto voxel_index = block->computeVoxelIndexFromCoordinates(point);
    if (!block->isValidVoxelIndex(voxel_index)) continue;
    auto& point_indices = point_map[block_index][voxel_index];
    auto& voxel = block->getVoxelByVoxelIndex(voxel_index);
    info.points[index].ever_free_level_dynamic = voxel.ever_free;
    if (point_indices.empty()) {
      voxel.last_lidar_occupied = frame_;
      voxel.clustering_processed = false;
      if (voxel.ever_free) seeds.emplace_back(block_index, voxel_index);
    }
    point_indices.push_back(index);
  }
  auto clusters = clustering_->performClustering(point_map, seeds, frame_, cloud, info);
  tracking_->track(cloud, clusters, info);
  ever_free_->updateEverFreeVoxels(frame_);
  const voxblox::Transformation transform(Eigen::Quaternionf(world_from_sensor.linear().cast<float>()),
                                         world_from_sensor.translation().cast<float>());
  const voxblox::Colors colors(local_points.size(), voxblox::Color(160, 160, 160));
  integrator_->integratePointCloud(transform, local_points, colors, false);

  for (std::size_t index = 0; index < info.points.size(); ++index) {
    if (info.points[index].ever_free_level_dynamic) result.seeds.push_back(world_points[original_indices[index]]);
    if (info.points[index].object_level_dynamic) result.labels[original_indices[index]] = PointLabel::Dynamic;
  }
  for (const auto& cluster : clusters) {
    DetectionCluster output;
    output.id = cluster.id;
    output.minimum = cluster.aabb.min_corner.getVector3fMap().cast<double>();
    output.maximum = cluster.aabb.max_corner.getVector3fMap().cast<double>();
    output.points = cluster.points.size();
    output.track_length = cluster.track_length;
    output.dynamic = cluster.valid;
    for (auto index : cluster.points) output.seed_points += info.points[index].ever_free_level_dynamic;
    result.clusters.push_back(output);
  }
  result.blocks = layer_->getNumberOfAllocatedBlocks();
  result.frame = frame_;
  return result;
}

std::vector<Eigen::Vector3d> DynabloxAdapter::static_map() const {
  std::vector<Eigen::Vector3d> result;
  voxblox::BlockIndexList indices;
  layer_->getAllAllocatedBlocks(&indices);
  for (const auto& index : indices) {
    const auto block = layer_->getBlockPtrByIndex(index);
    for (std::size_t voxel_index = 0; voxel_index < block->num_voxels(); ++voxel_index) {
      const auto& voxel = block->getVoxelByLinearIndex(voxel_index);
      if (voxel.weight > 1e-6 && std::abs(voxel.distance) < config_.resolution * 0.75 && !voxel.dynamic) {
        result.push_back(block->computeCoordinatesFromLinearIndex(voxel_index).cast<double>());
      }
    }
  }
  return result;
}

std::vector<Eigen::Vector3d> DynabloxAdapter::free_centers(std::size_t limit) const {
  std::vector<Eigen::Vector3d> result;
  voxblox::BlockIndexList indices;
  layer_->getAllAllocatedBlocks(&indices);
  for (const auto& index : indices) {
    const auto block = layer_->getBlockPtrByIndex(index);
    for (std::size_t voxel_index = 0; voxel_index < block->num_voxels(); ++voxel_index) {
      if (result.size() >= limit) return result;
      if (block->getVoxelByLinearIndex(voxel_index).ever_free) {
        result.push_back(block->computeCoordinatesFromLinearIndex(voxel_index).cast<double>());
      }
    }
  }
  return result;
}

}
