#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tsl/robin_hood.h>

#include "zhao_lio/types.h"

namespace zhao_lio {

// Super-LIO VoxelGridClosest: per voxel keep the original point closest to the voxel center.
// Points are never averaged, so each kept point keeps its own timestamp.
class VoxelDownsample {
 public:
  explicit VoxelDownsample(float leaf = 0.5f) { set_leaf(leaf); }
  void set_leaf(float leaf) { leaf_ = leaf; inv_leaf_ = 1.0f / leaf; }
  float leaf() const { return leaf_; }

  // 21 bits per axis: valid for |coord| < 2^20 * leaf.
  inline void key_of(const Eigen::Vector3f& p, uint64_t& key, float& d2) const {
    const Eigen::Vector3i idx = (p * inv_leaf_).array().round().cast<int>();
    d2 = (p - leaf_ * idx.cast<float>()).squaredNorm();
    constexpr int64_t kOff = int64_t(1) << 20;
    constexpr uint64_t kMask = (uint64_t(1) << 21) - 1;
    key = ((static_cast<uint64_t>(idx[0] + kOff) & kMask) << 42) |
          ((static_cast<uint64_t>(idx[1] + kOff) & kMask) << 21) |
          (static_cast<uint64_t>(idx[2] + kOff) & kMask);
  }

  void filter_serial(const std::vector<LidarPoint>& in, std::vector<LidarPoint>& out) {
    map_.clear();
    map_.reserve(in.size());
    best_d2_.clear();
    out.clear();
    for (const auto& pt : in) {
      uint64_t key;
      float d2;
      key_of(pt.xyz, key, d2);
      auto it = map_.find(key);
      if (it == map_.end()) {
        map_.emplace(key, static_cast<uint32_t>(out.size()));
        out.push_back(pt);
        best_d2_.push_back(d2);
      } else if (d2 < best_d2_[it->second]) {
        out[it->second] = pt;
        best_d2_[it->second] = d2;
      }
    }
  }

  // Parallel key computation + parallel sort by (key, d2, index), then take the head of each run.
  // Ties resolve to the lowest index, same as filter_serial.
  void filter_parallel(const std::vector<LidarPoint>& in, std::vector<LidarPoint>& out) const {
    struct Entry { uint64_t key; float d2; uint32_t idx; };
    std::vector<Entry> e(in.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, in.size(), 2048), [&](const tbb::blocked_range<size_t>& r) {
      for (size_t i = r.begin(); i < r.end(); ++i) {
        key_of(in[i].xyz, e[i].key, e[i].d2);
        e[i].idx = static_cast<uint32_t>(i);
      }
    });
    tbb::parallel_sort(e.begin(), e.end(), [](const Entry& a, const Entry& b) {
      if (a.key != b.key) return a.key < b.key;
      if (a.d2 != b.d2) return a.d2 < b.d2;
      return a.idx < b.idx;
    });
    out.clear();
    for (size_t i = 0; i < e.size(); ++i) {
      if (i == 0 || e[i].key != e[i - 1].key) out.push_back(in[e[i].idx]);
    }
  }

 private:
  float leaf_ = 0.5f;
  float inv_leaf_ = 2.0f;
  robin_hood::unordered_flat_map<uint64_t, uint32_t> map_;
  std::vector<float> best_d2_;
};

}  // namespace zhao_lio
