#include <gtest/gtest.h>

#include "zhao_lio/events.h"

using namespace zhao_lio;

TEST(Events, MergesByTimeImuFirstOnTies) {
  std::vector<LidarPoint> pts(3);
  pts[0].time = 0.1; pts[1].time = 0.2; pts[2].time = 0.3;
  std::vector<ImuSample> imus(2);
  imus[0].time = 0.2; imus[1].time = 0.35;

  const auto ev = merge_events(pts, imus);
  ASSERT_EQ(ev.size(), 5u);
  EXPECT_EQ(ev[0].type, Event::kPoint); EXPECT_EQ(ev[0].index, 0u);
  EXPECT_EQ(ev[1].type, Event::kImu);   EXPECT_EQ(ev[1].index, 0u);
  EXPECT_EQ(ev[2].type, Event::kPoint); EXPECT_EQ(ev[2].index, 1u);
  EXPECT_EQ(ev[3].type, Event::kPoint); EXPECT_EQ(ev[3].index, 2u);
  EXPECT_EQ(ev[4].type, Event::kImu);   EXPECT_EQ(ev[4].index, 1u);
  for (size_t i = 1; i < ev.size(); ++i) EXPECT_LE(ev[i - 1].time, ev[i].time);
}

TEST(Events, EmptyInputs) {
  EXPECT_TRUE(merge_events({}, {}).empty());
  std::vector<ImuSample> imus(2);
  EXPECT_EQ(merge_events({}, imus).size(), 2u);
}
