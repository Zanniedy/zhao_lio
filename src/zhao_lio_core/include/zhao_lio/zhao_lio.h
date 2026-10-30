#pragma once

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include <tbb/global_control.h>

#include "zhao_lio/eskf30.h"
#include "zhao_lio/oct_vox_map.hpp"
#include "zhao_lio/params.h"
#include "zhao_lio/types.h"
#include "zhao_lio/updater/updater.h"
#include "zhao_lio/voxel_downsample.h"

namespace zhao_lio {

struct FrameTiming {
  double preprocess_ms = 0, downsample_ms = 0, prematch_ms = 0, update_ms = 0, map_ms = 0, total_ms = 0;
};

struct ScanResult {
  double time = 0.0;  // scan end time
  State30 state;
  std::vector<Eigen::Vector3f> points_world;
  std::vector<std::pair<double, State30>> state_trace;
  FrameTiming timing;
  StrategyStats stats;
  std::size_t num_raw = 0;
  std::size_t num_downsampled = 0;
};

class ZhaoLio {
 public:
  enum class Status { kWaitImuInit, kWaitMapInit, kProcess };

  explicit ZhaoLio(const Params& p);
  // Feeds one synchronized measure; returns true and fills `out` once odometry is running.
  bool process(const Measure& m, ScanResult& out);
  Status status() const { return status_; }
  const OctVoxMap& map() const { return *map_; }
  const Eskf30& filter() const { return kf_; }

 private:
  void init_from_imu();
  void reset();
  void filter_and_downsample(const ScanFrame& scan, FrameTiming& timing);

  Params p_;
  Status status_ = Status::kWaitImuInit;
  Eskf30 kf_;
  std::unique_ptr<OctVoxMap> map_;
  std::unique_ptr<UpdateStrategy> strategy_;
  std::unique_ptr<tbb::global_control> tbb_limit_;
  VoxelDownsample ds_;
  std::vector<ImuSample> init_imus_;
  std::vector<LidarPoint> filtered_, down_;
  StrategyOutput strat_out_;
  int map_frames_ = 0;
};

}  // namespace zhao_lio
