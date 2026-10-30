#include <gtest/gtest.h>

#include "zhao_lio/events.h"
#include "zhao_lio/preprocess.h"
#include "zhao_lio/updater/updater.h"
#include "zhao_lio/voxel_downsample.h"
#include "sim_scene.h"

using namespace zhao_lio;

// Static sensor at the origin, filter starts 5 cm / 3 cm / 2 cm off; one scan should pull it back.
static void run_static_correction(StrategyType type) {
  Params prm;
  prm.strategy = type;
  OctVoxMap map(prm.map_resolution, prm.map_capacity);
  map.insert(sim::room_surface_points());

  sim::Trajectory tr;  // static until 1.5 s
  std::mt19937 rng(3);
  ScanFrame scan = sim::make_scan(tr, 0.0, 0.1, 4000, rng);
  std::vector<LidarPoint> pts;
  VoxelDownsample(0.3f).filter_serial(scan.points, pts);
  sort_by_time(pts);
  std::vector<ImuSample> imus;
  for (int i = 0; i <= 20; ++i) imus.push_back(sim::make_imu(tr, i * 0.005));
  const auto events = merge_events(pts, imus);

  Eskf30 kf(make_eskf_options(prm));
  kf.x.grav = Eigen::Vector3d(0, 0, -9.81);
  kf.x.acc = Eigen::Vector3d(0, 0, 9.81);
  kf.x.pos = Eigen::Vector3d(0.05, -0.03, 0.02);
  kf.init_time(0.0);

  auto strategy = make_strategy(prm);
  StrategyOutput out;
  strategy->process({pts, imus, events}, kf, map, out);

  EXPECT_EQ(out.points_world.size(), pts.size());
  EXPECT_EQ(out.stats.points_in, static_cast<int>(pts.size()));
  EXPECT_GT(out.stats.points_used, static_cast<int>(pts.size() / 2));
  EXPECT_LT(kf.x.pos.norm(), 0.01);
  EXPECT_LT(kf.x.vel.norm(), 0.05);
}

TEST(Strategy, PointwiseCorrectsOffset) { run_static_correction(StrategyType::kPointwise); }
TEST(Strategy, PrematchCorrectsOffset) { run_static_correction(StrategyType::kPointwisePrematch); }
