#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "zhao_lio/oct_vox_map.hpp"
#include "zhao_lio_backend/pose_graph.h"
#include "zhao_lio_backend/se3.h"

namespace zhao_lio_backend {

struct RegistrationParams {
  float coarse_resolution = 1.2f;  // m; reach of the coarse level ≈ 1.5 voxels
  float fine_resolution = 0.4f;
  double coarse_plane_thr = 0.25;  // m, max neighbor distance to the fitted plane
  double fine_plane_thr = 0.1;
  int max_iters = 20;              // per level
  double huber = 0.1;              // m, fine level (coarse uses 3x)
  double inlier_thr = 0.15;        // m, |point-to-plane| counted as inlier on the fine level
  double sigma_floor = 0.03;       // m, lower bound of the residual scale used for information
  double n_eff = 200.0;            // information is scaled down as if at most n_eff points were independent
  double rot_scale = 10.0;         // m per rad when comparing translation and rotation directions
  double max_sigma = 0.3;          // m (rad * rot_scale); weaker directions are not constrained
  double min_eig_ratio = 2e-3;     // ... and so are directions below this fraction of the strongest one
  // Floor on the standard deviation of every kept direction (m, rotation scaled by rot_scale). Point
  // residuals are not independent (map blur, scan pattern), so the raw Hessian is far too confident.
  double sigma_min = 0.1;
};

// A point cloud prepared for scan-to-map registration at two resolutions.
class TargetMap {
 public:
  explicit TargetMap(const RegistrationParams& p = RegistrationParams());
  void build(const std::vector<Eigen::Vector3f>& pts);
  bool empty() const { return num_points_ == 0; }
  std::size_t num_points() const { return num_points_; }
  const zhao_lio::OctVoxMap& coarse() const { return *coarse_; }
  const zhao_lio::OctVoxMap& fine() const { return *fine_; }

 private:
  std::unique_ptr<zhao_lio::OctVoxMap> coarse_, fine_;
  std::size_t num_points_ = 0;
};

struct RegistrationResult {
  bool converged = false;
  Pose T = Pose::Identity();    // target <- source
  int num_points = 0;
  int num_inliers = 0;
  double inlier_ratio = 0.0;
  double rmse = 0.0;            // m, over inliers
  Mat6 info = Mat6::Zero();     // on the right perturbation of T
  SqrtInfo sqrt_info;           // observable directions only
  int rank = 0;
};

// Projects `info` onto its well-constrained directions. Translation and rotation are compared after
// scaling rotation by rot_scale; a direction is kept when its standard deviation is below max_sigma
// and its eigenvalue is at least min_eig_ratio of the largest (a condition-number bound).
SqrtInfo observable_sqrt_info(const Mat6& info, const RegistrationParams& p, int* rank);

// Point-to-plane ICP of `source` (source frame) into `target`, starting from `init`.
RegistrationResult register_scan(const TargetMap& target, const std::vector<Eigen::Vector3f>& source,
                                 const Pose& init, const RegistrationParams& p, bool use_coarse);

}  // namespace zhao_lio_backend
