#pragma once

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

#include "zhao_lio/eskf30.h"
#include "zhao_lio/oct_vox_map.hpp"
#include "zhao_lio/params.h"
#include "zhao_lio/types.h"

namespace zhao_lio {

struct StrategyStats {
  int points_in = 0;
  int points_used = 0;   // points that produced a filter update
  int rematched = 0;     // prematch only: points re-searched because the predicted plane drifted
  double t_prematch_ms = 0.0;
  double t_update_ms = 0.0;
};

struct StrategyInput {
  const std::vector<LidarPoint>& points;  // sorted by time
  const std::vector<ImuSample>& imus;
  const std::vector<Event>& events;       // merge_events(points, imus)
};

struct StrategyOutput {
  std::vector<Eigen::Vector3f> points_world;            // one per input point, post-update pose
  std::vector<std::pair<double, State30>> state_trace;  // only with Params::record_pointwise_states
  StrategyStats stats;
};

class UpdateStrategy {
 public:
  explicit UpdateStrategy(const Params& p) : params_(p) {}
  virtual ~UpdateStrategy() = default;
  // The map is read-only for the whole call; the caller inserts points_world afterwards.
  virtual void process(const StrategyInput& in, Eskf30& kf, const OctVoxMap& map, StrategyOutput& out) = 0;

 protected:
  bool pass_gate(const Eigen::Vector3d& p_lidar, double r) const { return p_lidar.norm() > params_.match_s * r * r; }
  void reset_output(const StrategyInput& in, StrategyOutput& out) const;
  void record(double t, const Eskf30& kf, StrategyOutput& out) const {
    if (params_.record_pointwise_states) out.state_trace.emplace_back(t, kf.x);
  }
  Params params_;
};

// Shared IMU event: predict state and covariance to the IMU time, then IMU measurement update.
void imu_step(Eskf30& kf, const ImuSample& imu);
EskfOptions make_eskf_options(const Params& p);
std::unique_ptr<UpdateStrategy> make_strategy(const Params& p);

double ms_since(std::chrono::steady_clock::time_point t0);

}  // namespace zhao_lio
