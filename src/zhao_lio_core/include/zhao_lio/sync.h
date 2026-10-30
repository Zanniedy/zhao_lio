#pragma once

#include <cstddef>
#include <deque>

#include "zhao_lio/types.h"

namespace zhao_lio {

// Pairs each LiDAR scan with all IMU samples up to its end time.
class MeasureSync {
 public:
  static constexpr std::size_t kMaxScans = 20;

  void push_imu(const ImuSample& imu);  // out-of-order samples are dropped
  void push_scan(ScanFrame scan);       // keeps at most kMaxScans waiting scans
  // Pops the oldest scan once the newest IMU sample reaches its end time.
  bool pop(Measure& out);
  std::size_t dropped_scans() const { return dropped_scans_; }

 private:
  std::deque<ImuSample> imus_;
  std::deque<ScanFrame> scans_;
  double last_imu_time_ = -1e300;
  std::size_t dropped_scans_ = 0;
};

}  // namespace zhao_lio
