#include "zhao_lio_backend/scan_context.h"

#include <cmath>

namespace zhao_lio_backend {

// A point at azimuth a falls in sector floor((a + pi) / (2 pi) * sectors). If the candidate frame C
// and the query frame Q satisfy p_C = Rz(yaw) p_Q, query column k matches candidate column
// k + yaw / sector_width, which is the shift sc_distance returns. So ScMatch::yaw is the rotation
// R_CQ = Rz(yaw), and T_map_Q = T_map_C * Rz(yaw).
ScanContext ScanContext::make(const std::vector<Eigen::Vector3f>& leveled, const ScanContextParams& p) {
  ScanContext sc;
  sc.desc = Eigen::MatrixXf::Zero(p.rings, p.sectors);
  for (const auto& pt : leveled) {
    const double r = std::hypot(pt.x(), pt.y());
    if (r < 0.1 || r >= p.max_range) continue;
    const int ring = std::min(p.rings - 1, static_cast<int>(r / p.max_range * p.rings));
    const double a = std::atan2(pt.y(), pt.x()) + M_PI;
    const int sector = std::min(p.sectors - 1, static_cast<int>(a / (2 * M_PI) * p.sectors));
    const float h = std::max(0.0f, pt.z() + static_cast<float>(p.height_offset));
    sc.desc(ring, sector) = std::max(sc.desc(ring, sector), h);
  }
  sc.ring_key = sc.desc.rowwise().mean();
  return sc;
}

double sc_distance(const ScanContext& a, const ScanContext& b, int* best_shift) {
  const int S = static_cast<int>(a.desc.cols());
  // Column norms are shift independent; precompute.
  Eigen::VectorXf na(S), nb(S);
  for (int k = 0; k < S; ++k) { na[k] = a.desc.col(k).norm(); nb[k] = b.desc.col(k).norm(); }
  double best = 1.0;
  int arg = 0;
  for (int shift = 0; shift < S; ++shift) {
    double sum = 0.0;
    int cnt = 0;
    for (int k = 0; k < S; ++k) {
      const int kb = (k + shift) % S;
      if (na[k] < 1e-6f || nb[kb] < 1e-6f) continue;
      sum += a.desc.col(k).dot(b.desc.col(kb)) / (na[k] * nb[kb]);
      ++cnt;
    }
    if (cnt == 0) continue;
    // Few overlapping columns make the similarity unreliable; weight by overlap.
    const double d = 1.0 - (sum / cnt) * std::min(1.0, cnt / (0.25 * S));
    if (d < best) { best = d; arg = shift; }
  }
  if (best_shift) *best_shift = arg;
  return best;
}

}  // namespace zhao_lio_backend
