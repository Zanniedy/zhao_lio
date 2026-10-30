#pragma once
#include <vector>

#include "zhao_lio/plane.h"
#include "zhao_lio/updater/updater.h"

namespace zhao_lio {

// Phase 1: nominal-only prediction on a copy of the filter gives each point's predicted pose.
// Phase 2: KNN + plane fit for all points in parallel (TBB) against the read-only map.
// Phase 3: serial predict + rank-1 update; residual recomputed with the current state, and
//          the plane is re-searched when |r| > rematch_thr.
class PointwisePrematchStrategy : public UpdateStrategy {
 public:
  using UpdateStrategy::UpdateStrategy;
  void process(const StrategyInput& in, Eskf30& kf, const OctVoxMap& map, StrategyOutput& out) override;

 private:
  std::vector<Eigen::Vector3d> pred_world_;
  std::vector<Plane> planes_;
};

}  // namespace zhao_lio
