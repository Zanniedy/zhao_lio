#include <gtest/gtest.h>

#include <random>

#include "zhao_lio_backend/registration.h"
#include "zhao_lio_backend/scan_context.h"

using namespace zhao_lio_backend;

namespace {

// Room [-8,8]x[-6,6]x[-1,3] with a box obstacle; dense surface samples.
std::vector<Eigen::Vector3f> room(bool corridor) {
  std::vector<Eigen::Vector3f> v;
  const float X = corridor ? 200.0f : 8.0f, Y = corridor ? 1.5f : 6.0f, s = 0.05f;
  for (float a = -X; a <= X; a += s)
    for (float b = -1; b <= 3; b += s) { v.emplace_back(a, -Y, b); v.emplace_back(a, Y, b); }
  for (float a = -X; a <= X; a += s)
    for (float b = -Y; b <= Y; b += s) { v.emplace_back(a, b, -1); v.emplace_back(a, b, 3); }
  if (!corridor) {
    for (float a = -Y; a <= Y; a += s)
      for (float b = -1; b <= 3; b += s) { v.emplace_back(-X, a, b); v.emplace_back(X, a, b); }
    for (float a = 2; a <= 4; a += s)
      for (float b = -1; b <= 1; b += s) { v.emplace_back(a, 2, b); v.emplace_back(4, a - 2, b); }
  }
  return v;
}

std::vector<Eigen::Vector3f> sample_in_frame(const std::vector<Eigen::Vector3f>& world, const Pose& T_world_src,
                                             int n, std::mt19937& rng) {
  std::uniform_int_distribution<std::size_t> pick(0, world.size() - 1);
  std::normal_distribution<float> noise(0, 0.01f);
  const Pose inv = T_world_src.inverse();
  std::vector<Eigen::Vector3f> out;
  for (int k = 0; k < n; ++k) {
    const Eigen::Vector3f p = world[pick(rng)];
    if ((p.cast<double>() - T_world_src.translation()).norm() > 15) continue;
    out.push_back((inv * p.cast<double>()).cast<float>() + Eigen::Vector3f(noise(rng), noise(rng), noise(rng)));
  }
  return out;
}

}  // namespace

TEST(Registration, RecoversOffset) {
  std::mt19937 rng(7);
  const auto world = room(false);
  RegistrationParams p;
  TargetMap target(p);
  target.build(world);
  const Pose truth = pose_from(rot_z(0.3), {1.0, -0.5, 0.2});
  const auto src = sample_in_frame(world, truth, 4000, rng);
  Pose init = truth * se3_exp((Vec6() << 0.6, -0.5, 0.1, 0.0, 0.0, 0.08).finished());
  const RegistrationResult r = register_scan(target, src, init, p, true);
  EXPECT_TRUE(r.converged);
  EXPECT_LT((r.T.translation() - truth.translation()).norm(), 0.03);
  EXPECT_LT(so3_log(r.T.linear().transpose() * truth.linear()).norm(), 0.01);
  EXPECT_GT(r.inlier_ratio, 0.8);
  EXPECT_EQ(r.rank, 6);
}

TEST(Registration, CorridorDropsAxialDirection) {
  std::mt19937 rng(8);
  const auto world = room(true);
  RegistrationParams p;
  TargetMap target(p);
  target.build(world);
  const Pose truth = pose_from(Eigen::Matrix3d::Identity(), {0.0, 0.2, 0.0});
  const auto src = sample_in_frame(world, truth, 4000, rng);
  const RegistrationResult r = register_scan(target, src, truth, p, false);
  EXPECT_EQ(r.rank, 5);
  // The dropped direction is (almost) pure x translation.
  const Eigen::Matrix<double, 1, 6> ex = (Eigen::Matrix<double, 1, 6>() << 1, 0, 0, 0, 0, 0).finished();
  EXPECT_LT((r.sqrt_info * ex.transpose()).norm(), 1.0);
}

TEST(ScanContext, RecoversYaw) {
  std::mt19937 rng(9);
  const auto world = room(false);
  ScanContextParams p;
  const Pose c = pose_from(Eigen::Matrix3d::Identity(), {0.5, 0.5, 0});
  const double yaw = 1.0;
  const Pose q = pose_from(rot_z(yaw), {0.5, 0.5, 0});
  const ScanContext sc_c = ScanContext::make(sample_in_frame(world, c, 30000, rng), p);
  const ScanContext sc_q = ScanContext::make(sample_in_frame(world, q, 30000, rng), p);
  ScanContextDb db(p);
  db.add(sc_c);
  const ScMatch m = db.query(sc_q, [](int) { return true; });
  ASSERT_EQ(m.index, 0);
  EXPECT_LT(m.distance, 0.2);
  // R_CQ = Rz(yaw): T_map_Q = T_map_C * Rz(m.yaw)
  double err = std::remainder(m.yaw - yaw, 2 * M_PI);
  EXPECT_LT(std::abs(err), 2 * 2 * M_PI / p.sectors);
}
