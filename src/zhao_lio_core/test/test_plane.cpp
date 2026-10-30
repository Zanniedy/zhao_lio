#include <gtest/gtest.h>

#include <random>

#include "zhao_lio/plane.h"

using namespace zhao_lio;

static KnnHeap5 heap_of(const std::vector<Eigen::Vector3f>& pts) {
  KnnHeap5 h;
  for (const auto& p : pts) h.try_insert(p.squaredNorm(), p);
  return h;
}

TEST(Plane, RecoversNoisyPlane) {
  std::mt19937 rng(0);
  std::normal_distribution<float> nz(0.0f, 0.01f);
  const auto h = heap_of({{0, 0, 1 + nz(rng)}, {0.3f, 0, 1 + nz(rng)}, {0, 0.3f, 1 + nz(rng)},
                          {0.3f, 0.3f, 1 + nz(rng)}, {0.15f, 0.1f, 1 + nz(rng)}});
  const Plane pl = fit_plane(h, 0.1);
  ASSERT_TRUE(pl.valid);
  EXPECT_NEAR(std::abs(pl.n.z()), 1.0, 1e-3);
  EXPECT_NEAR(pl.n.norm(), 1.0, 1e-12);
  EXPECT_NEAR(pl.n.dot(Eigen::Vector3d(0.1, 0.1, 1.0)) + pl.d, 0.0, 0.02);
}

TEST(Plane, RejectsNonPlanarAndTooFewPoints) {
  EXPECT_FALSE(fit_plane(heap_of({{0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {0, 0, 2}, {1, 1, 0}}), 0.1).valid);
  EXPECT_FALSE(fit_plane(heap_of({{0, 0, 1}, {1, 0, 1}, {0, 1, 1}}), 0.1).valid);
}

TEST(Plane, MatchPlaneOnMap) {
  OctVoxMap map(0.5f, 100000);
  std::vector<Eigen::Vector3f> wall;
  for (float y = -2; y <= 2; y += 0.05f)
    for (float z = -2; z <= 2; z += 0.05f) wall.emplace_back(3.0f, y, z);
  map.insert(wall);
  const Plane pl = match_plane(map, Eigen::Vector3d(3.02, 0.1, 0.2), 0.1);
  ASSERT_TRUE(pl.valid);
  EXPECT_NEAR(std::abs(pl.n.x()), 1.0, 1e-3);
  EXPECT_NEAR(std::abs(pl.n.dot(Eigen::Vector3d(3.0, 0, 0)) + pl.d), 0.0, 1e-3);
}
