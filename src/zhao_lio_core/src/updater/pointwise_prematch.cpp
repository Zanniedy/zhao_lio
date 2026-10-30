#include "zhao_lio/updater/pointwise_prematch.h"

#include <cmath>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace zhao_lio {

void PointwisePrematchStrategy::process(const StrategyInput& in, Eskf30& kf, const OctVoxMap& map,
                                        StrategyOutput& out) {
  reset_output(in, out);
  const size_t n = in.points.size();
  pred_world_.resize(n);
  planes_.assign(n, Plane());

  const auto t0 = std::chrono::steady_clock::now();
  {  // Phase 1: nominal prediction on a copy; IMU samples overwrite ω and a with bias-corrected readings.
    Eskf30 pred = kf;
    for (const Event& ev : in.events) {
      if (ev.type == Event::kImu) {
        const ImuSample& imu = in.imus[ev.index];
        pred.predict_state(imu.time);
        pred.x.omg = imu.gyro - pred.x.bg;
        pred.x.acc = imu.acc - pred.x.ba;
      } else {
        pred.predict_state(ev.time);
        pred_world_[ev.index] = pred.x.lidar_to_world(in.points[ev.index].xyz.cast<double>());
      }
    }
  }
  // Phase 2: parallel association.
  tbb::parallel_for(tbb::blocked_range<size_t>(0, n, 64), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t i = r.begin(); i < r.end(); ++i) planes_[i] = match_plane(map, pred_world_[i], params_.plane_thr);
  });
  out.stats.t_prematch_ms = ms_since(t0);

  // Phase 3: serial point-wise update.
  const auto t1 = std::chrono::steady_clock::now();
  for (const Event& ev : in.events) {
    if (ev.type == Event::kImu) {
      imu_step(kf, in.imus[ev.index]);
      continue;
    }
    const Eigen::Vector3d p = in.points[ev.index].xyz.cast<double>();
    if (ev.time >= kf.time()) {
      kf.predict_state(ev.time);
      Plane pl = planes_[ev.index];
      if (pl.valid) {
        double r = kf.point_residual(pl.n, pl.d, p);
        if (std::abs(r) > params_.rematch_thr) {
          ++out.stats.rematched;
          pl = match_plane(map, kf.x.lidar_to_world(p), params_.plane_thr);
          if (pl.valid) r = kf.point_residual(pl.n, pl.d, p);
        }
        if (pl.valid && pass_gate(p, r) && kf.update_point(pl.n, pl.d, p)) {
          ++out.stats.points_used;
          record(ev.time, kf, out);
        }
      }
    }
    out.points_world[ev.index] = kf.x.lidar_to_world(p).cast<float>();
  }
  out.stats.t_update_ms = ms_since(t1);
}

}  // namespace zhao_lio
