#include <gtest/gtest.h>

#include <random>

#include "zhao_lio_backend/chi2.h"
#include "zhao_lio_backend/pose_graph.h"
#include "zhao_lio_backend/se3.h"

using namespace zhao_lio_backend;

TEST(Se3, ExpLogRoundTrip) {
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0, 0.8);
  for (int k = 0; k < 100; ++k) {
    Vec6 xi;
    for (int i = 0; i < 6; ++i) xi[i] = n(rng);
    EXPECT_LT((se3_log(se3_exp(xi)) - xi).norm(), 1e-9);
  }
}

TEST(Se3, AdjointIdentity) {
  Vec6 a, b;
  a << 0.3, -1, 2, 0.2, 0.4, -0.5;
  b << 0.01, 0.02, -0.03, 0.004, -0.002, 0.001;
  const Pose T = se3_exp(a);
  const Pose lhs = T * se3_exp(b) * T.inverse();
  const Pose rhs = se3_exp(se3_adjoint(T) * b);
  EXPECT_LT((lhs.matrix() - rhs.matrix()).norm(), 1e-9);
}

TEST(Chi2, KnownQuantiles) {
  EXPECT_NEAR(chi2_quantile(1, 0.99), 6.635, 0.15);
  EXPECT_NEAR(chi2_quantile(3, 0.99), 11.345, 0.1);
  EXPECT_NEAR(chi2_quantile(6, 0.99), 16.812, 0.1);
  EXPECT_NEAR(chi2_quantile(6, 0.9999), 27.856, 0.03 * 27.856);  // Wilson-Hilferty is ~2% high here
}

// Square loop: odometry drifts in yaw, one loop closure pulls the end back to the start.
TEST(PoseGraph, LoopClosureRemovesDrift) {
  std::vector<Pose> gt;
  for (int k = 0; k < 40; ++k) {
    const int side = k / 10, s = k % 10;
    Pose T = Pose::Identity();
    const double yaw = side * M_PI / 2;
    Eigen::Vector3d c[4] = {{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}};
    T.linear() = rot_z(yaw);
    T.translation() = c[side] + rot_z(yaw) * Eigen::Vector3d(s, 0, 0);
    gt.push_back(T);
  }
  const Pose drift = se3_exp((Vec6() << 0.01, 0, 0, 0, 0, 0.01).finished());
  PoseGraph g;
  Pose est = gt[0];
  g.add_node(0, est);
  g.add_prior(0, gt[0], sqrt_info_from_sigmas(Vec6::Constant(1e-3)));
  const SqrtInfo Lo = sqrt_info_from_sigmas((Vec6() << 0.05, 0.05, 0.05, 0.01, 0.01, 0.01).finished());
  for (int k = 1; k < 40; ++k) {
    const Pose z = gt[k - 1].inverse() * gt[k] * drift;
    est = est * z;
    g.add_node(k, est);
    g.add_between(k - 1, k, z, Lo);
  }
  const double before = (g.pose(39).translation() - gt[39].translation()).norm();
  const Pose zl = gt[0].inverse() * gt[39];
  const SqrtInfo Ll = sqrt_info_from_sigmas(Vec6::Constant(0.01));
  const Innovation inn = g.innovation_between(0, 39, zl, Ll);
  ASSERT_TRUE(inn.valid);
  EXPECT_EQ(inn.dof, 6);
  g.add_between(0, 39, zl, Ll);
  g.optimize(20);
  const double after = (g.pose(39).translation() - gt[39].translation()).norm();
  EXPECT_GT(before, 1.0);
  EXPECT_LT(after, 0.05);
  double worst = 0;
  for (int k = 0; k < 40; ++k) worst = std::max(worst, (g.pose(k).translation() - gt[k].translation()).norm());
  EXPECT_LT(worst, 0.6);
}

// Marginalizing old nodes of a chain with absolute factors must keep the newest estimate.
TEST(PoseGraph, MarginalizationKeepsSolution) {
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0, 0.05);
  auto noisy = [&](const Pose& T) {
    Vec6 e;
    for (int i = 0; i < 6; ++i) e[i] = n(rng);
    return T * se3_exp(e);
  };
  PoseGraph full, win;
  const SqrtInfo Lo = sqrt_info_from_sigmas(Vec6::Constant(0.02));
  const SqrtInfo La = sqrt_info_from_sigmas(Vec6::Constant(0.1));
  Pose prev = Pose::Identity();
  for (int k = 0; k < 30; ++k) {
    Pose T = Pose::Identity();
    T.translation() = Eigen::Vector3d(k, 0.1 * k, 0);
    T.linear() = rot_z(0.05 * k);
    const Pose meas = noisy(T);
    for (PoseGraph* g : {&full, &win}) {
      g->add_node(k, T);
      if (k > 0) g->add_between(k - 1, k, prev.inverse() * T, Lo);
      g->add_prior(k, meas, La);
      g->optimize(10);
    }
    while (win.size() > 8) { win.marginalize_oldest(); win.optimize(10); }
    prev = T;
  }
  EXPECT_EQ(win.size(), 8u);
  EXPECT_LT((full.pose(29).translation() - win.pose(29).translation()).norm(), 0.01);
  EXPECT_LT(so3_log(full.pose(29).linear().transpose() * win.pose(29).linear()).norm(), 1e-3);
  Eigen::MatrixXd Pf, Pw;
  ASSERT_TRUE(full.marginal({29}, Pf));
  ASSERT_TRUE(win.marginal({29}, Pw));
  EXPECT_NEAR(Pf.trace(), Pw.trace(), 0.1 * Pf.trace());
}

TEST(PoseGraph, PartialRankPrior) {
  PoseGraph g;
  g.add_node(0, Pose::Identity());
  g.add_prior(0, Pose::Identity(), sqrt_info_from_sigmas(Vec6::Constant(0.01)));
  Pose T = Pose::Identity();
  T.translation() = Eigen::Vector3d(1, 1, 0);
  g.add_node(1, T);
  g.add_between(0, 1, T, sqrt_info_from_sigmas((Vec6() << 1, 1, 1, 0.01, 0.01, 0.01).finished()));
  // Observation only along x: should move x but leave y at the odometry value.
  Pose z = T;
  z.translation() = Eigen::Vector3d(2, 5, 0);
  SqrtInfo L = SqrtInfo::Zero(1, 6);
  L(0, 0) = 100.0;
  g.add_prior(1, z, L);
  g.optimize(20);
  EXPECT_NEAR(g.pose(1).translation().x(), 2.0, 5e-3);
  EXPECT_NEAR(g.pose(1).translation().y(), 1.0, 5e-3);
}
