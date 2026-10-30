#include "zhao_lio/preprocess.h"

#include <algorithm>

namespace zhao_lio {

void preprocess(const ScanFrame& in, const Params& p, std::vector<LidarPoint>& out) {
  out.clear();
  out.reserve(in.points.size());
  const double min2 = p.blind_min * p.blind_min;
  const double max2 = p.blind_max * p.blind_max;
  const int step = std::max(1, p.point_filter_num);
  for (size_t i = 0; i < in.points.size(); i += step) {
    const LidarPoint& pt = in.points[i];
    if (!pt.xyz.allFinite()) continue;
    const double r2 = pt.xyz.squaredNorm();
    if (r2 < min2 || r2 > max2) continue;
    out.push_back(pt);
  }
}

void sort_by_time(std::vector<LidarPoint>& pts) {
  std::sort(pts.begin(), pts.end(), [](const LidarPoint& a, const LidarPoint& b) { return a.time < b.time; });
}

}  // namespace zhao_lio
