#include <gtest/gtest.h>

#include "zhao_lio/sync.h"

using namespace zhao_lio;

static ImuSample imu_at(double t) { ImuSample s; s.time = t; return s; }
static ScanFrame scan(double b, double e) { ScanFrame f; f.t_begin = b; f.t_end = e; return f; }

TEST(Sync, WaitsForImuCoveringScanEnd) {
  MeasureSync sync;
  for (int i = 0; i <= 50; ++i) sync.push_imu(imu_at(i * 0.01));  // 0.00 .. 0.50
  sync.push_scan(scan(0.4, 0.5));
  sync.push_scan(scan(0.5, 0.6));
  Measure m;
  ASSERT_TRUE(sync.pop(m));
  EXPECT_DOUBLE_EQ(m.scan.t_end, 0.5);
  EXPECT_EQ(m.imus.size(), 51u);
  EXPECT_FALSE(sync.pop(m));                 // no IMU up to 0.6 yet
  for (int i = 51; i <= 60; ++i) sync.push_imu(imu_at(i * 0.01));
  ASSERT_TRUE(sync.pop(m));
  EXPECT_EQ(m.imus.size(), 10u);
  EXPECT_GT(m.imus.front().time, 0.5);
}

TEST(Sync, DropsOutOfOrderImuAndOldScans) {
  MeasureSync sync;
  sync.push_imu(imu_at(1.0));
  sync.push_imu(imu_at(0.5));                // out of order, dropped
  for (int i = 0; i < 25; ++i) sync.push_scan(scan(2.0 + i, 2.1 + i));
  EXPECT_EQ(sync.dropped_scans(), 5u);       // buffer keeps 20
  sync.push_imu(imu_at(100.0));
  Measure m;
  ASSERT_TRUE(sync.pop(m));
  EXPECT_DOUBLE_EQ(m.scan.t_begin, 7.0);
  EXPECT_EQ(m.imus.size(), 1u);              // only the 1.0 sample is <= 7.1
}
