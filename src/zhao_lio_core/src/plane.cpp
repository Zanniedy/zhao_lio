#include "zhao_lio/plane.h"

#include <cmath>

#include <Eigen/QR>

namespace zhao_lio {

Plane fit_plane(const KnnHeap5& nn, double thr) {
  Plane pl;
  const int n = nn.count;
  if (n < 4) return pl;
  Eigen::Matrix<double, Eigen::Dynamic, 3, 0, 5, 3> A(n, 3);
  Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 5, 1> b(n);
  for (int j = 0; j < n; ++j) {
    A.row(j) = nn.points[j].cast<double>().transpose();
    b(j) = -1.0;
  }
  const Eigen::Vector3d nv = A.colPivHouseholderQr().solve(b);
  const double norm = nv.norm();
  if (norm < 1e-6) return pl;
  pl.n = nv / norm;
  pl.d = 1.0 / norm;
  for (int j = 0; j < n; ++j) {
    if (std::abs(pl.n.dot(nn.points[j].cast<double>()) + pl.d) > thr) return pl;
  }
  pl.valid = true;
  return pl;
}

Plane match_plane(const OctVoxMap& map, const Eigen::Vector3d& p_world, double thr) {
  KnnHeap5 nn;
  map.get_top_k(p_world.cast<float>(), nn);
  return fit_plane(nn, thr);
}

}  // namespace zhao_lio
