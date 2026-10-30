#include <gtest/gtest.h>

#include <cmath>

#include "zhao_lio/preprocess.h"

using namespace zhao_lio;

TEST(Preprocess, RangeFilterNthAndNan) {
  ScanFrame f;
  for (int i = 0; i < 10; ++i) {
    LidarPoint p; p.xyz = Eigen::Vector3f(1.0f + i, 0, 0); p.time = i; f.points.push_back(p);
  }
  f.points[4].xyz.x() = NAN;
  Params prm; prm.blind_min = 1.5; prm.blind_max = 8.5; prm.point_filter_num = 2;
  std::vector<LidarPoint> out;
  preprocess(f, prm, out);
  // every 2nd point: indices 0,2,4,6,8 (ranges 1,3,NaN,7,9); 1 and 9 out of range, NaN dropped
  ASSERT_EQ(out.size(), 2u);
  EXPECT_DOUBLE_EQ(out[0].time, 2); EXPECT_DOUBLE_EQ(out[1].time, 6);
}

TEST(Preprocess, SortByTime) {
  std::vector<LidarPoint> v(3);
  v[0].time = 3; v[1].time = 1; v[2].time = 2;
  sort_by_time(v);
  EXPECT_DOUBLE_EQ(v[0].time, 1); EXPECT_DOUBLE_EQ(v[2].time, 3);
}
