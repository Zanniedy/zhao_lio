#include <gtest/gtest.h>

#include "zhao_lio/eskf30.h"

using namespace zhao_lio;
using V3 = Eigen::Vector3d;

static Eskf30 static_filter(EskfOptions opt = EskfOptions()) {
  Eskf30 kf(opt);
  kf.x.grav = V3(0, 0, -9.81);
  kf.x.acc = V3(0, 0, 9.81);
  kf.init_time(0.0);
  return kf;
}

TEST(Eskf, StaticImuDoesNotDiverge) {
  Eskf30 kf = static_filter();
  for (int i = 1; i <= 1000; ++i) {
    const double t = i * 0.005;
    kf.predict_state(t);
    kf.predict_cov(t);
    ASSERT_TRUE(kf.update_imu(V3::Zero(), V3(0, 0, 9.81)));
  }
  EXPECT_LT(kf.x.pos.norm(), 1e-9);
  EXPECT_LT(kf.x.vel.norm(), 1e-9);
  EXPECT_TRUE(kf.P.allFinite());
  EXPECT_LT((kf.P - kf.P.transpose()).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(Eskf, PointUpdateReducesResidual) {
  Eskf30 kf = static_filter();
  kf.x.pos = V3(0, 0, 0.05);                    // true pos is 0; floor plane z = -1
  const V3 n(0, 0, 1);
  const double d = 1.0;
  const V3 pts[] = {{1, 0, -1}, {0, 1, -1}, {-1, 0.5, -1}, {0.5, -1, -1}};
  const double r0 = std::abs(kf.point_residual(n, d, pts[0]));
  for (const auto& p : pts) ASSERT_TRUE(kf.update_point(n, d, p));
  EXPECT_LT(std::abs(kf.point_residual(n, d, pts[0])), 0.5 * r0);
}

TEST(Eskf, GyroSaturationDoesNotBlockAccelerometer) {
  Eskf30 kf = static_filter();
  const V3 omg0 = kf.x.omg, bg0 = kf.x.bg;
  ASSERT_TRUE(kf.update_imu(V3(40.0, 0, 0), V3(1.0, 0, 9.81)));   // gyro x saturated
  EXPECT_EQ(kf.x.omg.x(), omg0.x());
  EXPECT_EQ(kf.x.bg.x(), bg0.x());
  EXPECT_GT(kf.x.acc.x(), 0.1);                                    // accelerometer x still updates
}

TEST(Eskf, AccSaturationDoesNotBlockGyro) {
  Eskf30 kf = static_filter();
  const double acc_x0 = kf.x.acc.x(), ba_x0 = kf.x.ba.x();
  ASSERT_TRUE(kf.update_imu(V3(0.5, 0, 0), V3(40.0, 0, 9.81)));    // acc x saturated
  EXPECT_EQ(kf.x.acc.x(), acc_x0);
  EXPECT_EQ(kf.x.ba.x(), ba_x0);
  EXPECT_GT(kf.x.omg.x(), 0.1);
}

TEST(Eskf, FixedExtrinsicStaysFixed) {
  Eskf30 kf = static_filter();
  kf.x.R_LI = so3_exp(V3(0, 0, 0.1));
  kf.x.t_LI = V3(0.1, 0, 0);
  const Eigen::Matrix3d R0 = kf.x.R_LI;
  const V3 t0 = kf.x.t_LI;
  kf.x.pos = V3(0.02, -0.03, 0.05);
  for (int i = 0; i < 50; ++i) {
    kf.predict_cov(0.01 * (i + 1));
    kf.update_point(V3(0, 0, 1), 1.0, V3(1, 0.1 * i, -1));
    kf.update_point(V3(1, 0, 0), -3.0, V3(3, 0.1 * i, 0));
  }
  EXPECT_EQ(kf.x.R_LI, R0);
  EXPECT_EQ(kf.x.t_LI, t0);
}
