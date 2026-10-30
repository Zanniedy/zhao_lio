#pragma once

#include <array>
#include <cstdint>
#include <list>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <tsl/robin_map.h>

#include "zhao_lio/hknn_list60.h"

namespace zhao_lio {

struct KnnHeap5 {
  using Point = Eigen::Vector3f;
  uint8_t count = 0;
  uint8_t worst = 0;
  float max_dist2 = 0.0f;
  std::array<float, 5> dist2{};
  std::array<Point, 5> points;

  void reset() { count = 0; worst = 0; max_dist2 = 0.0f; }

  inline void try_insert(float d2, const Point& pt) {
    if (count < 5) {
      dist2[count] = d2;
      points[count] = pt;
      if (count == 0 || d2 > max_dist2) { max_dist2 = d2; worst = count; }
      ++count;
      return;
    }
    if (d2 >= max_dist2) return;
    dist2[worst] = d2;
    points[worst] = pt;
    update_worst();
  }

 private:
  inline void update_worst() {
    const float d0 = dist2[0], d1 = dist2[1], d2 = dist2[2], d3 = dist2[3], d4 = dist2[4];
    const uint8_t i01 = d0 > d1 ? 0 : 1;
    const float m01 = d0 > d1 ? d0 : d1;
    const uint8_t i23 = d2 > d3 ? 2 : 3;
    const float m23 = d2 > d3 ? d2 : d3;
    const uint8_t i0123 = m01 > m23 ? i01 : i23;
    const float m0123 = m01 > m23 ? m01 : m23;
    worst = m0123 > d4 ? i0123 : 4;
    max_dist2 = m0123 > d4 ? m0123 : d4;
  }
};

// One voxel = 2x2x2 sub-voxels, each holding one running-mean representative point.
struct OctVox {
  static constexpr uint8_t kEmpty = 0;
  static constexpr uint8_t kMaxPoints = 20;
  static constexpr float kMergeDist2 = 0.1f * 0.1f;

  std::array<uint8_t, 8> counts{};
  std::array<Eigen::Vector3f, 8> points;

  OctVox(const Eigen::Vector3f& pt, uint8_t local_idx) { counts.fill(kEmpty); points[local_idx] = pt; counts[local_idx] = 1; }

  void add(const Eigen::Vector3f& pt, uint8_t local_idx) {
    uint8_t& c = counts[local_idx];
    Eigen::Vector3f& s = points[local_idx];
    if (c == kEmpty) { s = pt; c = 1; return; }
    if (c >= kMaxPoints) return;
    if ((pt - s).squaredNorm() > kMergeDist2) return;
    s = (s * c + pt) / static_cast<float>(c + 1);
    ++c;
  }

  bool get(uint8_t local_idx, Eigen::Vector3f& pt) const {
    if (counts[local_idx] == kEmpty) return false;
    pt = points[local_idx];
    return true;
  }
};
class OctVoxMap {
 public:
  using Point = Eigen::Vector3f;
  using Key = Eigen::Vector3i;

  OctVoxMap(float resolution, std::size_t capacity)
      : resolution_(resolution), sub_inv_resolution_(2.0f / resolution), capacity_(capacity) {
    for (std::size_t i = 0; i < flat_search_order_offsets.size(); ++i) {
      group_ptrs_.push_back(flat_search_order.data() + flat_search_order_offsets[i]);
    }
    group_max_ = static_cast<int>(flat_search_order_offsets.size()) - 1;
    // Super-LIO's table assumes 0.25 m sub-voxels; distances scale linearly with the sub-voxel size.
    const float s = (0.5f * resolution) / 0.25f;
    for (std::size_t i = 0; i < min_dis2_.size(); ++i) min_dis2_[i] = orders_min_dis2[i] * s * s;
  }

  float resolution() const { return resolution_; }
  std::size_t num_voxels() const { return data_.size(); }
  void clear() { grids_.clear(); data_.clear(); }

  void insert(const std::vector<Point>& pts) {
    for (const auto& pt : pts) {
      Key key;
      uint8_t local_idx;
      split_key(pt, key, local_idx);
      auto it = grids_.find(key);
      if (it == grids_.end()) {
        data_.emplace_front(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(pt, local_idx));
        grids_.insert(std::make_pair(key, data_.begin()));
        if (data_.size() >= capacity_) {
          grids_.erase(data_.back().first);
          data_.pop_back();
        }
      } else {
        it->second->second.add(pt, local_idx);
        data_.splice(data_.begin(), data_, it->second);
      }
    }
  }

  void get_points(std::vector<Point>& out) const {
    out.clear();
    Point p;
    for (const auto& kv : data_) {
      for (uint8_t i = 0; i < 8; ++i) {
        if (kv.second.get(i, p)) out.push_back(p);
      }
    }
  }
  // 5 nearest stored points using Super-LIO's precomputed sub-voxel search order with early stop.
  void get_top_k(const Point& q, KnnHeap5& top) const {
    top.reset();
    const Key fine = (q * sub_inv_resolution_).array().floor().template cast<int>();
    const Key key(fine[0] >> 1, fine[1] >> 1, fine[2] >> 1);
    const int dx = fine[0] & 1, dy = fine[1] & 1, dz = fine[2] & 1;
    const uint8_t local_idx = static_cast<uint8_t>((dz << 2) | (dy << 1) | dx);
    const Key mirror(1 - (dx << 1), 1 - (dy << 1), 1 - (dz << 1));

    constexpr int kPre = 8;
    const OctVox* pre[kPre] = {nullptr};
    for (int i = 0; i < kPre; ++i) {
      auto it = grids_.find(key + mirror.cwiseProduct(HKNN_neighbor_voxel[i]));
      if (it != grids_.end()) pre[i] = &it->second->second;
    }

    Point sub;
    for (int g = 0; g < group_max_; ++g) {
      const uint8_t* it = group_ptrs_[g];
      const uint8_t* end = group_ptrs_[g + 1];
      while (it < end) {
        const uint8_t neighbor = *it++;
        uint8_t n = *it++;
        const OctVox* vox = nullptr;
        if (neighbor < kPre) {
          vox = pre[neighbor];
        } else {
          auto f = grids_.find(key + mirror.cwiseProduct(HKNN_neighbor_voxel[neighbor]));
          if (f != grids_.end()) vox = &f->second->second;
        }
        if (!vox) { it += n; continue; }
        while (n--) {
          const uint8_t li = static_cast<uint8_t>((*it++) ^ local_idx);
          if (vox->get(li, sub)) top.try_insert((sub - q).squaredNorm(), sub);
        }
      }
      if (top.count == 5 && top.max_dist2 < min_dis2_[g]) break;
    }
  }

 private:
  struct KeyHash {
    std::size_t operator()(const Key& v) const {
      std::size_t h = static_cast<std::size_t>(v[0]);
      h ^= v[1] * 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= v[2] * 0x85ebca6b + (h << 6) + (h >> 2);
      return h;
    }
  };
  using DataList = std::list<std::pair<Key, OctVox>>;

  inline void split_key(const Point& pt, Key& key, uint8_t& local_idx) const {
    const Key fine = (pt * sub_inv_resolution_).array().floor().template cast<int>();
    key = Key(fine[0] >> 1, fine[1] >> 1, fine[2] >> 1);
    local_idx = static_cast<uint8_t>(((fine[2] & 1) << 2) | ((fine[1] & 1) << 1) | (fine[0] & 1));
  }

  float resolution_;
  float sub_inv_resolution_;
  std::size_t capacity_;
  DataList data_;  // front = most recently touched
  tsl::robin_map<Key, DataList::iterator, KeyHash> grids_;
  std::vector<const uint8_t*> group_ptrs_;
  std::array<float, 6> min_dis2_{};
  int group_max_ = 0;
};

}  // namespace zhao_lio
