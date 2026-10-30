#include <gtest/gtest.h>

#include <algorithm>
#include <random>

#include <tbb/parallel_for.h>

#include "zhao_lio/oct_vox_map.hpp"

using namespace zhao_lio;
using V3f = Eigen::Vector3f;

static std::vector<V3f> dense_cube(int n, float half, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> u(-half, half);
  std::vector<V3f> v(n);
  for (auto& p : v) p = V3f(u(rng), u(rng), u(rng));
  return v;
}

static std::vector<float> sorted_d2(const KnnHeap5& h) {
  std::vector<float> d(h.dist2.begin(), h.dist2.begin() + h.count);
  std::sort(d.begin(), d.end());
  return d;
}

// The early-stop table must stay exact at resolutions other than Super-LIO's 0.5 m.
static void check_against_brute_force(float res) {
  const float half = 12.0f * res;
  OctVoxMap map(res, 1000000);
  map.insert(dense_cube(400000, half, 1));
  std::vector<V3f> stored;
  map.get_points(stored);

  std::mt19937 rng(7);
  std::uniform_real_distribution<float> u(-0.6f * half, 0.6f * half);
  for (int q = 0; q < 300; ++q) {
    const V3f query(u(rng), u(rng), u(rng));
    KnnHeap5 h;
    map.get_top_k(query, h);
    ASSERT_EQ(h.count, 5);
    std::vector<float> bf;
    for (const auto& p : stored) bf.push_back((p - query).squaredNorm());
    std::partial_sort(bf.begin(), bf.begin() + 5, bf.end());
    const auto got = sorted_d2(h);
    for (int k = 0; k < 5; ++k) ASSERT_NEAR(got[k], bf[k], 1e-6f) << "res=" << res << " q=" << q << " k=" << k;
  }
}

TEST(OctVoxMap, KnnMatchesBruteForceRes05) { check_against_brute_force(0.5f); }
TEST(OctVoxMap, KnnMatchesBruteForceRes02) { check_against_brute_force(0.2f); }

TEST(OctVoxMap, ConcurrentQueriesMatchSerial) {
  OctVoxMap map(0.5f, 1000000);
  map.insert(dense_cube(200000, 5.0f, 3));
  const auto queries = dense_cube(5000, 3.0f, 4);
  std::vector<std::vector<float>> serial(queries.size()), parallel(queries.size());
  for (size_t i = 0; i < queries.size(); ++i) { KnnHeap5 h; map.get_top_k(queries[i], h); serial[i] = sorted_d2(h); }
  tbb::parallel_for(size_t(0), queries.size(), [&](size_t i) { KnnHeap5 h; map.get_top_k(queries[i], h); parallel[i] = sorted_d2(h); });
  EXPECT_EQ(serial, parallel);
}

TEST(OctVoxMap, CapacityEvictsOldestVoxels) {
  OctVoxMap map(1.0f, 10);
  std::vector<V3f> pts;
  for (int i = 0; i < 20; ++i) pts.emplace_back(0.5f + 3.0f * i, 0.5f, 0.5f);
  map.insert(pts);
  EXPECT_LE(map.num_voxels(), 10u);
  KnnHeap5 h;
  map.get_top_k(pts.back(), h);           // newest survives
  ASSERT_GE(h.count, 1);
  EXPECT_NEAR(sorted_d2(h)[0], 0.0f, 1e-9f);
}
