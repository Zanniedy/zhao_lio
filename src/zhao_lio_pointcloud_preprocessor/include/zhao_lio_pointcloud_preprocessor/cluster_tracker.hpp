#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace zhao_lio_pointcloud_preprocessor {

struct ClusterConfig {
  double tolerance = 0.35;
  double ground_z = 0.0;
  double ground_clearance = 0.12;
  double max_extent = 4.0;
  double association_distance = 0.70;
  double association_ambiguity = 0.12;
  double motion_window = 0.30;
  double min_displacement = 0.10;
  double min_speed = 0.25;
  double max_speed = 6.0;
  double fit_distance = 0.30;
  double max_fit_rmse = 0.12;
  double min_fit_improvement = 0.025;
  double min_overlap = 0.65;
  double track_timeout = 0.40;
  int min_points = 8;
  int confirmation_frames = 2;
};

struct ClusterView {
  std::uint32_t id = 0;
  Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
  Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
  double speed = 0.0;
  double displacement = 0.0;
  double static_rmse = 0.0;
  double motion_rmse = 0.0;
  std::size_t points = 0;
  int confirmations = 0;
  bool moving = false;
  bool removed = false;
  std::string state = "warming";
};

struct ClusterResult {
  std::vector<bool> remove;
  std::vector<ClusterView> clusters;
  std::size_t rejected_components = 0;
};

class ClusterTracker {
 public:
  explicit ClusterTracker(ClusterConfig config = {});
  ClusterResult update(const std::vector<Eigen::Vector3d>& points, double time,
                       const Eigen::Isometry3d& current_to_previous,
                       const Eigen::Isometry3d& lidar_to_base);
  void reset();

 private:
  struct Observation {
    double time = 0.0;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d extent = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> points;
  };
  struct Track {
    std::uint32_t id = 0;
    std::deque<Observation> history;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    int confirmations = 0;
  };
  ClusterConfig config_;
  std::vector<Track> tracks_;
  Eigen::Isometry3d world_from_lidar_ = Eigen::Isometry3d::Identity();
  double last_time_ = -1.0;
  std::uint32_t next_id_ = 1;
};

}
