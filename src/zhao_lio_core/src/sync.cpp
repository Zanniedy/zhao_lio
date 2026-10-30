#include "zhao_lio/sync.h"

namespace zhao_lio {

void MeasureSync::push_imu(const ImuSample& imu) {
  if (imu.time <= last_imu_time_) return;
  last_imu_time_ = imu.time;
  imus_.push_back(imu);
}

void MeasureSync::push_scan(ScanFrame scan) {
  scans_.push_back(std::move(scan));
  while (scans_.size() > kMaxScans) {
    scans_.pop_front();
    ++dropped_scans_;
  }
}

bool MeasureSync::pop(Measure& out) {
  if (scans_.empty() || imus_.empty()) return false;
  if (imus_.back().time < scans_.front().t_end) return false;
  out.scan = std::move(scans_.front());
  scans_.pop_front();
  out.imus.clear();
  while (!imus_.empty() && imus_.front().time <= out.scan.t_end) {
    out.imus.push_back(imus_.front());
    imus_.pop_front();
  }
  return true;
}

}  // namespace zhao_lio
