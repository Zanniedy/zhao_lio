#pragma once

#include <vector>

#include "zhao_lio/params.h"
#include "zhao_lio/types.h"

namespace zhao_lio {

// Keeps every point_filter_num-th point whose range is in [blind_min, blind_max] and is finite.
void preprocess(const ScanFrame& in, const Params& p, std::vector<LidarPoint>& out);
void sort_by_time(std::vector<LidarPoint>& pts);

}  // namespace zhao_lio
