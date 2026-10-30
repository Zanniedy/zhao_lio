#include <gtest/gtest.h>

#include <cmath>

#include "zhao_lio/zhao_lio.h"
#include "zhao_lio/sync.h"
#include "sim_scene.h"

using namespace zhao_lio;

struct RunResult { double pos_err; double yaw_err_deg; int used; int rematched; int frames; State30 final_state; };

static RunResult run(StrategyType type) {
  Params prm;
  prm.strategy = type;
  prm.downsample_size = 0.3f;
  prm.blind_min = 0.3;
  ZhaoLio lio(prm);
  MeasureSync sync;
  sim::Trajectory tr;
  std::mt19937 rng(42);
  const double imu_dt = 0.005, scan_dt = 0.1, t_total = 8.0;
  int imu_i = 0;
  RunResult rr{0, 0, 0, 0, 0, State30()};
  ScanResult res, last;
  for (int s = 0; s < static_cast<int>(t_total / scan_dt); ++s) {
    ScanFrame scan = sim::make_scan(tr, s * scan_dt, scan_dt, 4000, rng);
    while (imu_i * imu_dt <= scan.t_end + 1e-9) { sync.push_imu(sim::make_imu(tr, imu_i * imu_dt)); ++imu_i; }
    sync.push_scan(std::move(scan));
    Measure m;
    while (sync.pop(m)) {
      if (lio.process(m, res)) {
        ++rr.frames; rr.used += res.stats.points_used; rr.rematched += res.stats.rematched;
        last = res;
      }
    }
  }
  rr.final_state = last.state;
  rr.pos_err = (last.state.pos - tr.pos(last.time)).norm();
  const Eigen::Matrix3d dR = tr.rot(last.time).transpose() * last.state.rot;
  rr.yaw_err_deg = std::abs(std::atan2(dR(1, 0), dR(0, 0))) * 180.0 / M_PI;
  return rr;
}

TEST(Pipeline, PointwiseTracksTrajectory) {
  const auto r = run(StrategyType::kPointwise);
  EXPECT_GT(r.frames, 60);
  EXPECT_LT(r.pos_err, 0.1);
  EXPECT_LT(r.yaw_err_deg, 1.0);
}

TEST(Pipeline, PrematchTracksTrajectoryAndRarelyRematches) {
  const auto r = run(StrategyType::kPointwisePrematch);
  EXPECT_GT(r.frames, 60);
  EXPECT_LT(r.pos_err, 0.1);
  EXPECT_LT(r.yaw_err_deg, 1.0);
  ASSERT_GT(r.used, 0);
  EXPECT_LT(static_cast<double>(r.rematched) / r.used, 0.01);
}

TEST(Pipeline, StrategiesAgree) {
  const auto a = run(StrategyType::kPointwise);
  const auto b = run(StrategyType::kPointwisePrematch);
  EXPECT_LT((a.final_state.pos - b.final_state.pos).norm(), 0.05);
}
