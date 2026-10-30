#pragma once

#include <Eigen/Core>

#include "zhao_lio/oct_vox_map.hpp"

namespace zhao_lio {

struct Plane {
  Eigen::Vector3d n = Eigen::Vector3d::Zero();  // unit normal
  double d = 0.0;                               // n·p + d = 0
  bool valid = false;
};

// Super-LIO calc_plane_coeff: least squares A n = -1 over 4–5 neighbors, then every
// neighbor must lie within thr of the plane.
Plane fit_plane(const KnnHeap5& nn, double thr);

// KNN in the map around p_world, then fit_plane.
Plane match_plane(const OctVoxMap& map, const Eigen::Vector3d& p_world, double thr);

}  // namespace zhao_lio
