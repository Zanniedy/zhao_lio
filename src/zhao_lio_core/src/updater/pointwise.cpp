#include "zhao_lio/updater/pointwise.h"

#include "zhao_lio/plane.h"

namespace zhao_lio {

void PointwiseStrategy::process(const StrategyInput& in, Eskf30& kf, const OctVoxMap& map, StrategyOutput& out) {
  const auto t0 = std::chrono::steady_clock::now();
  reset_output(in, out);
  for (const Event& ev : in.events) {
    if (ev.type == Event::kImu) {
      imu_step(kf, in.imus[ev.index]);
      continue;
    }
    const Eigen::Vector3d p = in.points[ev.index].xyz.cast<double>();
    if (ev.time >= kf.time()) {  // points older than the filter are only mapped, not used
      kf.predict_state(ev.time);
      const Plane pl = match_plane(map, kf.x.lidar_to_world(p), params_.plane_thr);
      if (pl.valid && pass_gate(p, kf.point_residual(pl.n, pl.d, p)) && kf.update_point(pl.n, pl.d, p)) {
        ++out.stats.points_used;
        record(ev.time, kf, out);
      }
    }
    out.points_world[ev.index] = kf.x.lidar_to_world(p).cast<float>();
  }
  out.stats.t_update_ms = ms_since(t0);
}

}  // namespace zhao_lio
