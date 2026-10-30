#include <gtest/gtest.h>

#include <map>
#include <random>

#include "zhao_lio/voxel_downsample.h"

using namespace zhao_lio;

static std::vector<LidarPoint> random_cloud(int n, float half, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> u(-half, half);
  std::vector<LidarPoint> v(n);
  for (int i = 0; i < n; ++i) { v[i].xyz = {u(rng), u(rng), u(rng)}; v[i].time = i * 1e-5; }
  return v;
}

TEST(Downsample, OnePointPerVoxelClosestToCenterTimestampsKept) {
  const auto in = random_cloud(20000, 20.0f, 1);
  VoxelDownsample ds(0.5f);
  std::vector<LidarPoint> out;
  ds.filter_serial(in, out);
  ASSERT_GT(out.size(), 1000u);

  std::map<uint64_t, float> best;  // brute force: min d2 per voxel
  for (const auto& p : in) {
    uint64_t k; float d2; ds.key_of(p.xyz, k, d2);
    auto it = best.find(k);
    if (it == best.end() || d2 < it->second) best[k] = d2;
  }
  EXPECT_EQ(out.size(), best.size());

  std::map<double, Eigen::Vector3f> by_time;
  for (const auto& p : in) by_time[p.time] = p.xyz;
  std::map<uint64_t, int> seen;
  for (const auto& p : out) {
    uint64_t k; float d2; ds.key_of(p.xyz, k, d2);
    EXPECT_EQ(++seen[k], 1);
    EXPECT_FLOAT_EQ(d2, best[k]);
    ASSERT_TRUE(by_time.count(p.time));        // original point, original timestamp
    EXPECT_EQ(by_time[p.time], p.xyz);
  }
}

TEST(Downsample, ParallelMatchesSerial) {
  const auto in = random_cloud(50000, 30.0f, 2);
  VoxelDownsample ds(0.3f);
  std::vector<LidarPoint> a, b;
  ds.filter_serial(in, a);
  ds.filter_parallel(in, b);
  ASSERT_EQ(a.size(), b.size());
  auto by_t = [](const LidarPoint& x, const LidarPoint& y) { return x.time < y.time; };
  std::sort(a.begin(), a.end(), by_t);
  std::sort(b.begin(), b.end(), by_t);
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_DOUBLE_EQ(a[i].time, b[i].time);
    EXPECT_EQ(a[i].xyz, b[i].xyz);
  }
}

TEST(Downsample, NegativeAndFarCoordinatesDoNotCollide) {
  VoxelDownsample ds(0.5f);
  std::vector<LidarPoint> in(2);
  in[0].xyz = {-100.0f, 0, 0};
  in[1].xyz = {100.0f, 0, 0};
  std::vector<LidarPoint> out;
  ds.filter_serial(in, out);
  EXPECT_EQ(out.size(), 2u);
}
