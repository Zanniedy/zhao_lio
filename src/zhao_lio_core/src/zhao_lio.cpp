#include "zhao_lio/zhao_lio.h"

#include <chrono>
#include <cstdio>

#include <Eigen/Geometry>

#include "zhao_lio/events.h"
#include "zhao_lio/preprocess.h"

namespace zhao_lio {

using Clock = std::chrono::steady_clock;

ZhaoLio::ZhaoLio(const Params& p)
    : p_(p),
      kf_(make_eskf_options(p)),
      map_(std::make_unique<OctVoxMap>(p.map_resolution, p.map_capacity)),
      strategy_(make_strategy(p)),
      ds_(p.downsample_size > 0 ? p.downsample_size : 1.0f) {
  if (p.num_threads > 0) {
    tbb_limit_ = std::make_unique<tbb::global_control>(tbb::global_control::max_allowed_parallelism, p.num_threads);
  }
}

void ZhaoLio::reset() {
  status_ = Status::kWaitImuInit;
  map_->clear();
  map_frames_ = 0;
  init_imus_.clear();
  kf_ = Eskf30(make_eskf_options(p_));
}

// Static start assumed: gravity alignment makes world z point up, mean gyro is the gyro bias.
void ZhaoLio::init_from_imu() {
  Eigen::Vector3d mg = Eigen::Vector3d::Zero(), ma = Eigen::Vector3d::Zero();
  for (const auto& s : init_imus_) { mg += s.gyro; ma += s.acc; }
  mg /= static_cast<double>(init_imus_.size());
  ma /= static_cast<double>(init_imus_.size());

  kf_ = Eskf30(make_eskf_options(p_));
  kf_.x.rot = Eigen::Quaterniond::FromTwoVectors(ma.normalized(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  kf_.x.grav = Eigen::Vector3d(0, 0, -p_.gravity_norm);
  kf_.x.acc = ma.normalized() * p_.gravity_norm;
  kf_.x.bg = mg;
  kf_.x.R_LI = p_.extrinsic_R;
  kf_.x.t_LI = p_.extrinsic_T;
  kf_.init_time(init_imus_.back().time);
  init_imus_.clear();
}

void ZhaoLio::filter_and_downsample(const ScanFrame& scan, FrameTiming& timing) {
  auto t = Clock::now();
  preprocess(scan, p_, filtered_);
  timing.preprocess_ms = ms_since(t);
  t = Clock::now();
  if (p_.downsample_size <= 0) {
    down_ = filtered_;
  } else if (p_.downsample_parallel) {
    ds_.filter_parallel(filtered_, down_);
  } else {
    ds_.filter_serial(filtered_, down_);
  }
  sort_by_time(down_);
  timing.downsample_ms = ms_since(t);
}
bool ZhaoLio::process(const Measure& m, ScanResult& out) {
  if (status_ == Status::kWaitImuInit) {
    init_imus_.insert(init_imus_.end(), m.imus.begin(), m.imus.end());
    if (!init_imus_.empty() && static_cast<int>(init_imus_.size()) >= p_.imu_init_num) {
      init_from_imu();
      status_ = Status::kWaitMapInit;
    }
    return false;
  }

  if (status_ == Status::kWaitMapInit) {
    for (const auto& imu : m.imus) imu_step(kf_, imu);
    FrameTiming unused;
    filter_and_downsample(m.scan, unused);
    std::vector<Eigen::Vector3f> world(down_.size());
    for (size_t i = 0; i < down_.size(); ++i) world[i] = kf_.x.lidar_to_world(down_[i].xyz.cast<double>()).cast<float>();
    map_->insert(world);
    if (++map_frames_ >= p_.init_map_frames) status_ = Status::kProcess;
    return false;
  }

  const auto t_total = Clock::now();
  out.timing = FrameTiming();
  out.num_raw = m.scan.points.size();
  filter_and_downsample(m.scan, out.timing);
  out.num_downsampled = down_.size();

  const std::vector<Event> events = merge_events(down_, m.imus);
  strategy_->process({down_, m.imus, events}, kf_, *map_, strat_out_);
  kf_.predict_state(m.scan.t_end);

  if (!kf_.x.pos.allFinite() || !kf_.x.rot.allFinite() || !kf_.x.vel.allFinite()) {
    std::fprintf(stderr, "[zhao_lio] state became non-finite, resetting\n");
    reset();
    return false;
  }

  const auto t_map = Clock::now();
  map_->insert(strat_out_.points_world);
  out.timing.map_ms = ms_since(t_map);
  out.timing.prematch_ms = strat_out_.stats.t_prematch_ms;
  out.timing.update_ms = strat_out_.stats.t_update_ms;
  out.timing.total_ms = ms_since(t_total);

  out.time = m.scan.t_end;
  out.state = kf_.x;
  out.stats = strat_out_.stats;
  out.points_world.swap(strat_out_.points_world);
  out.state_trace.swap(strat_out_.state_trace);
  return true;
}

}  // namespace zhao_lio
