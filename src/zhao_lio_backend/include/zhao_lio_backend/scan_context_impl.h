#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

namespace zhao_lio_backend {

template <class Pred>
std::vector<ScMatch> ScanContextDb::query_k(const ScanContext& q, int k, Pred allowed) const {
  // Ring-key prefilter, then full column-shift comparison.
  std::vector<std::pair<float, int>> pre;
  for (std::size_t i = 0; i < db_.size(); ++i) {
    if (!allowed(static_cast<int>(i))) continue;
    pre.emplace_back((db_[i].ring_key - q.ring_key).squaredNorm(), static_cast<int>(i));
  }
  const std::size_t n = std::min<std::size_t>(pre.size(), static_cast<std::size_t>(std::max(p_.candidates, k)));
  std::partial_sort(pre.begin(), pre.begin() + n, pre.end());
  std::vector<ScMatch> out;
  for (std::size_t c = 0; c < n; ++c) {
    int shift = 0;
    ScMatch m;
    m.index = pre[c].second;
    m.distance = sc_distance(q, db_[m.index], &shift);
    m.yaw = 2.0 * M_PI * shift / p_.sectors;
    out.push_back(m);
  }
  std::sort(out.begin(), out.end(), [](const ScMatch& a, const ScMatch& b) { return a.distance < b.distance; });
  if (static_cast<int>(out.size()) > k) out.resize(k);
  return out;
}

template <class Pred>
ScMatch ScanContextDb::query(const ScanContext& q, Pred allowed) const {
  const std::vector<ScMatch> v = query_k(q, 1, allowed);
  return v.empty() ? ScMatch() : v.front();
}

}  // namespace zhao_lio_backend
