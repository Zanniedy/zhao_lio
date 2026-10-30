#include "zhao_lio/events.h"

namespace zhao_lio {

std::vector<Event> merge_events(const std::vector<LidarPoint>& points, const std::vector<ImuSample>& imus) {
  std::vector<Event> events;
  events.reserve(points.size() + imus.size());
  size_t i = 0, j = 0;
  while (i < points.size() || j < imus.size()) {
    const bool take_imu = j < imus.size() && (i >= points.size() || imus[j].time <= points[i].time);
    if (take_imu) {
      events.push_back({Event::kImu, imus[j].time, static_cast<uint32_t>(j)});
      ++j;
    } else {
      events.push_back({Event::kPoint, points[i].time, static_cast<uint32_t>(i)});
      ++i;
    }
  }
  return events;
}

}  // namespace zhao_lio
