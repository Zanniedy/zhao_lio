#pragma once

#include <vector>

#include "zhao_lio/types.h"

namespace zhao_lio {

// Merges time-sorted points and IMU samples into one time-ordered event list.
// On equal timestamps the IMU sample comes first.
std::vector<Event> merge_events(const std::vector<LidarPoint>& points, const std::vector<ImuSample>& imus);

}  // namespace zhao_lio
