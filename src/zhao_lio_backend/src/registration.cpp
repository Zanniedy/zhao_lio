#include "zhao_lio_backend/registration.h"

#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>


namespace zhao_lio_backend {

namespace {

constexpr std::size_t kUnbounded = std::numeric_limits<std::size_t>::max() / 4;

struct Normal {
  Mat6 H = Mat6::Zero();
  Vec6 b = Vec6::Zero();
  int matched = 0, inliers = 0;
  double sq_in = 0.0;
  void add(const Normal& o) {
    H += o.H; b += o.b; matched += o.matched; inliers += o.inliers; sq_in += o.sq_in;
  }
};

struct Plane3 { Eigen::Vector3d n; double d; };

// PCA plane through the neighbors. Rejects near-collinear neighbor sets: their "normal" is
// arbitrary and would add information along directions the scene does not constrain.
bool fit_plane_pca(const zhao_lio::KnnHeap5& nn, double thr, Plane3& pl) {
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  for (int k = 0; k < nn.count; ++k) c += nn.points[k].cast<double>();
  c /= nn.count;
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (int k = 0; k < nn.count; ++k) {
    const Eigen::Vector3d d = nn.points[k].cast<double>() - c;
    C += d * d.transpose();
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C / nn.count);
  const Eigen::Vector3d l = es.eigenvalues();  // ascending
  if (l[1] < 0.05 * l[2]) return false;
  if (l[0] > 0.0625 * thr * thr) return false;  // spread along the normal must be well below thr
  pl.n = es.eigenvectors().col(0);
  pl.d = -pl.n.dot(c);
  for (int k = 0; k < nn.count; ++k)
    if (std::abs(pl.n.dot(nn.points[k].cast<double>()) + pl.d) > thr) return false;
  return true;
}

// One Gauss-Newton linearization over all source points.
Normal accumulate(const zhao_lio::OctVoxMap& map, float res, double plane_thr, double huber, double inlier_thr,
                  const std::vector<Eigen::Vector3f>& src, const Pose& T) {
  const Eigen::Matrix3d R = T.linear();
  const Eigen::Vector3d t = T.translation();
  const float reach2 = (1.5f * res) * (1.5f * res);
  return tbb::parallel_reduce(
      tbb::blocked_range<std::size_t>(0, src.size(), 256), Normal(),
      [&](const tbb::blocked_range<std::size_t>& r, Normal acc) {
        zhao_lio::KnnHeap5 nn;
        for (std::size_t k = r.begin(); k != r.end(); ++k) {
          const Eigen::Vector3d s = src[k].cast<double>();
          const Eigen::Vector3d p = R * s + t;
          map.get_top_k(p.cast<float>(), nn);
          if (nn.count < 5 || nn.max_dist2 > reach2) continue;
          Plane3 pl;
          if (!fit_plane_pca(nn, plane_thr, pl)) continue;
          const double e = pl.n.dot(p) + pl.d;
          Eigen::Matrix<double, 1, 6> J;
          J.head<3>() = pl.n.transpose() * R;
          J.tail<3>() = -pl.n.transpose() * R * skew(s);
          const double ae = std::abs(e);
          const double w = ae <= huber ? 1.0 : huber / ae;
          acc.H += w * J.transpose() * J;
          acc.b += w * J.transpose() * e;
          ++acc.matched;
          if (ae < inlier_thr) { ++acc.inliers; acc.sq_in += e * e; }
        }
        return acc;
      },
      [](Normal a, const Normal& b) { a.add(b); return a; });
}

bool solve_level(const zhao_lio::OctVoxMap& map, float res, double plane_thr, double huber, double inlier_thr,
                 int max_iters, const std::vector<Eigen::Vector3f>& src, Pose& T, Normal& last) {
  bool converged = false;
  for (int it = 0; it < max_iters; ++it) {
    last = accumulate(map, res, plane_thr, huber, inlier_thr, src, T);
    if (last.matched < 30) return false;
    const Mat6 A = last.H + 1e-6 * Mat6::Identity() * (1.0 + last.H.trace() / 6.0);
    const Vec6 dx = A.ldlt().solve(-last.b);
    if (!dx.allFinite()) return false;
    T = normalized(T * se3_exp(dx));
    if (dx.head<3>().norm() < 1e-4 && dx.tail<3>().norm() < 1e-5) { converged = true; break; }
  }
  last = accumulate(map, res, plane_thr, huber, inlier_thr, src, T);
  return converged || last.matched >= 30;
}

}  // namespace

TargetMap::TargetMap(const RegistrationParams& p)
    : coarse_(std::make_unique<zhao_lio::OctVoxMap>(p.coarse_resolution, kUnbounded)),
      fine_(std::make_unique<zhao_lio::OctVoxMap>(p.fine_resolution, kUnbounded)) {}

void TargetMap::build(const std::vector<Eigen::Vector3f>& pts) {
  coarse_->clear();
  fine_->clear();
  coarse_->insert(pts);
  fine_->insert(pts);
  num_points_ = pts.size();
}

SqrtInfo observable_sqrt_info(const Mat6& info, const RegistrationParams& p, int* rank) {
  const double rot_scale = p.rot_scale, max_sigma = p.max_sigma;
  Vec6 s;
  s << 1, 1, 1, 1 / rot_scale, 1 / rot_scale, 1 / rot_scale;
  const Mat6 Hs = s.asDiagonal() * info * s.asDiagonal();
  const Eigen::SelfAdjointEigenSolver<Mat6> es(0.5 * (Hs + Hs.transpose()));
  const double min_lambda = 1.0 / (max_sigma * max_sigma);
  const double lmax = es.eigenvalues().maxCoeff();
  std::vector<int> keep;
  for (int k = 0; k < 6; ++k) {
    const double l = es.eigenvalues()[k];
    if (l >= min_lambda && l >= p.min_eig_ratio * lmax) keep.push_back(k);
  }
  SqrtInfo L(static_cast<int>(keep.size()), 6);
  for (std::size_t r = 0; r < keep.size(); ++r) {
    const int k = keep[r];
    // Hs = S H S with S = diag(s), so H = S^-1 Hs S^-1 and each kept row is sqrt(l) v' S^-1.
    const double l = 1.0 / (1.0 / es.eigenvalues()[k] + p.sigma_min * p.sigma_min);
    L.row(static_cast<int>(r)) = std::sqrt(l) * es.eigenvectors().col(k).transpose() *
                                 s.cwiseInverse().asDiagonal();
  }
  if (rank) *rank = static_cast<int>(keep.size());
  return L;
}

RegistrationResult register_scan(const TargetMap& target, const std::vector<Eigen::Vector3f>& source,
                                 const Pose& init, const RegistrationParams& p, bool use_coarse) {
  RegistrationResult out;
  out.T = init;
  out.num_points = static_cast<int>(source.size());
  if (target.empty() || source.size() < 50) return out;
  Normal last;
  if (use_coarse) {
    solve_level(target.coarse(), p.coarse_resolution, p.coarse_plane_thr, 3 * p.huber, 3 * p.inlier_thr, p.max_iters,
                source, out.T, last);
  }
  out.converged = solve_level(target.fine(), p.fine_resolution, p.fine_plane_thr, p.huber, p.inlier_thr, p.max_iters,
                              source, out.T, last);
  out.num_inliers = last.inliers;
  out.inlier_ratio = out.num_points > 0 ? static_cast<double>(last.inliers) / out.num_points : 0.0;
  out.rmse = last.inliers > 0 ? std::sqrt(last.sq_in / last.inliers) : 0.0;
  const double sigma = std::max(out.rmse, p.sigma_floor);
  const double inflation = std::max(1.0, last.matched / p.n_eff);
  out.info = last.H / (sigma * sigma * inflation);
  out.sqrt_info = observable_sqrt_info(out.info, p, &out.rank);
  return out;
}

}  // namespace zhao_lio_backend
