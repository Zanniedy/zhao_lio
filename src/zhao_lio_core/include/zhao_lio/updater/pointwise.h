#pragma once
#include "zhao_lio/updater/updater.h"

namespace zhao_lio {

// Baseline: per point predict -> KNN -> plane -> rank-1 update, fully serial (small_point_lio style).
class PointwiseStrategy : public UpdateStrategy {
 public:
  using UpdateStrategy::UpdateStrategy;
  void process(const StrategyInput& in, Eskf30& kf, const OctVoxMap& map, StrategyOutput& out) override;
};

}  // namespace zhao_lio
