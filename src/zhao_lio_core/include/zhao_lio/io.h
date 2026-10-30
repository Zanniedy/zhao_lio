#pragma once

#include <fstream>
#include <iomanip>
#include <string>

#include <Eigen/Geometry>

#include "zhao_lio/zhao_lio.h"

namespace zhao_lio {

// TUM format: t x y z qx qy qz qw (IMU pose in the odometry frame).
class TumWriter {
 public:
  bool open(const std::string& path) { f_.open(path); f_ << std::fixed << std::setprecision(9); return f_.good(); }
  bool is_open() const { return f_.is_open(); }
  void write(double t, const State30& s) {
    if (!f_.is_open()) return;
    const Eigen::Quaterniond q(s.rot);
    f_ << t << ' ' << s.pos.x() << ' ' << s.pos.y() << ' ' << s.pos.z() << ' '
       << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
 private:
  std::ofstream f_;
};

class TimingCsv {
 public:
  bool open(const std::string& path) {
    f_.open(path);
    f_ << "time,num_raw,num_down,points_used,rematched,preprocess_ms,downsample_ms,prematch_ms,update_ms,map_ms,total_ms\n";
    f_ << std::fixed << std::setprecision(6);
    return f_.good();
  }
  void write(const ScanResult& r) {
    if (!f_.is_open()) return;
    const auto& t = r.timing;
    f_ << r.time << ',' << r.num_raw << ',' << r.num_downsampled << ',' << r.stats.points_used << ','
       << r.stats.rematched << ',' << t.preprocess_ms << ',' << t.downsample_ms << ',' << t.prematch_ms << ','
       << t.update_ms << ',' << t.map_ms << ',' << t.total_ms << '\n';
  }
 private:
  std::ofstream f_;
};

}  // namespace zhao_lio
