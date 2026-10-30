#include "zhao_lio/updater/pointwise.h"
#include "zhao_lio/updater/pointwise_prematch.h"

namespace zhao_lio {

double ms_since(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void imu_step(Eskf30& kf, const ImuSample& imu) {
  kf.predict_state(imu.time);
  kf.predict_cov(imu.time);
  kf.update_imu(imu.gyro, imu.acc);
}

void UpdateStrategy::reset_output(const StrategyInput& in, StrategyOutput& out) const {
  out.points_world.assign(in.points.size(), Eigen::Vector3f::Zero());
  out.state_trace.clear();
  out.stats = StrategyStats();
  out.stats.points_in = static_cast<int>(in.points.size());
}

EskfOptions make_eskf_options(const Params& p) {
  EskfOptions o;
  o.laser_point_cov = p.laser_point_cov;
  o.imu_meas_omg_cov = p.imu_meas_omg_cov;
  o.imu_meas_acc_cov = p.imu_meas_acc_cov;
  o.vel_cov = p.vel_cov; o.omg_cov = p.omg_cov; o.acc_cov = p.acc_cov;
  o.bg_cov = p.bg_cov; o.ba_cov = p.ba_cov;
  o.check_satu = p.check_satu; o.satu_gyro = p.satu_gyro; o.satu_acc = p.satu_acc;
  o.extrinsic_est = p.extrinsic_est_en;
  return o;
}

std::unique_ptr<UpdateStrategy> make_strategy(const Params& p) {
  switch (p.strategy) {
    case StrategyType::kPointwise: return std::make_unique<PointwiseStrategy>(p);
    case StrategyType::kPointwisePrematch: return std::make_unique<PointwisePrematchStrategy>(p);
  }
  return nullptr;
}

}  // namespace zhao_lio
