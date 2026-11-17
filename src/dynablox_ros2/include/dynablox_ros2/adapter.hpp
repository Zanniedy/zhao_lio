#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <dynablox/processing/clustering.h>
#include <dynablox/processing/ever_free_integrator.h>
#include <dynablox/processing/tracking.h>
#include <voxblox/integrator/tsdf_integrator.h>

namespace dynablox_ros2 {

struct DynabloxConfig {
  double resolution = 0.20;
  double min_range = 0.50;
  double max_range = 20.0;
  double map_radius = 30.0;
  double max_time_gap = 1.0;
  int voxels_per_side = 16;
  int max_blocks = 3000;
  int num_threads = 4;
  std::string integration_method = "merged";
  dynablox::EverFreeIntegrator::Config ever_free;
  dynablox::Clustering::Config clustering;
  dynablox::Tracking::Config tracking;
};

enum class PointLabel { Static, Dynamic, Unobserved };

struct DetectionCluster {
  int id = 0;
  Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
  Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
  std::size_t points = 0;
  std::size_t seed_points = 0;
  int track_length = 0;
  bool dynamic = false;
};

struct DetectionResult {
  std::vector<PointLabel> labels;
  std::vector<Eigen::Vector3d> seeds;
  std::vector<DetectionCluster> clusters;
  std::size_t blocks = 0;
  int frame = 0;
  std::string reason = "ok";
};

class DynabloxAdapter {
 public:
  explicit DynabloxAdapter(DynabloxConfig config = {});
  DetectionResult update(const std::vector<Eigen::Vector3d>& world_points,
                         const Eigen::Isometry3d& world_from_sensor, double time);
  std::vector<Eigen::Vector3d> static_map() const;
  std::vector<Eigen::Vector3d> free_centers(std::size_t limit) const;
  void reset();
  dynablox::TsdfLayer::Ptr layer() const { return layer_; }

 private:
  DynabloxConfig config_;
  dynablox::TsdfLayer::Ptr layer_;
  std::unique_ptr<dynablox::EverFreeIntegrator> ever_free_;
  std::unique_ptr<dynablox::Clustering> clustering_;
  std::unique_ptr<dynablox::Tracking> tracking_;
  voxblox::TsdfIntegratorBase::Ptr integrator_;
  int frame_ = 0;
  double last_time_ = -1.0;
};

}
