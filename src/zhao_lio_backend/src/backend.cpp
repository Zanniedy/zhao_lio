#include "zhao_lio_backend/backend.h"

#include <chrono>
#include <cstdarg>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "zhao_lio/io.h"
#include "zhao_lio_backend/chi2.h"
#include "zhao_lio_backend/map_io.h"

namespace zhao_lio_backend {

struct Backend::Keyframe {
  int id = 0;
  double time = 0.0;
  int session = 0;
  Pose T_odom = Pose::Identity();
  Pose T_map = Pose::Identity();
  std::vector<Eigen::Vector3f> cloud;  // body frame
  Eigen::Vector3d up_body = Eigen::Vector3d::UnitZ();  // gravity up in the body frame (front-end estimate)
  ScanContext sc;
};

namespace {
// Rotation taking body coordinates to a gravity-leveled frame with the same origin (minimal rotation).
Eigen::Matrix3d leveling(const Eigen::Vector3d& up_body) {
  return Eigen::Quaterniond::FromTwoVectors(up_body, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
}  // namespace

const char* to_string(State s) {
  switch (s) {
    case State::kInitializing: return "INITIALIZING";
    case State::kNormal: return "NORMAL";
    case State::kObsDegraded: return "OBS_DEGRADED";
    case State::kSuspect: return "SUSPECT";
    case State::kLost: return "LOST";
  }
  return "?";
}

Mode parse_mode(const std::string& s) {
  if (s == "mapping") return Mode::kMapping;
  if (s == "localization") return Mode::kLocalization;
  throw std::invalid_argument("unknown backend mode: " + s + " (mapping | localization)");
}

namespace {

std::vector<Eigen::Vector3f> transform(const std::vector<Eigen::Vector3f>& pts, const Pose& T) {
  const Eigen::Matrix3f R = T.linear().cast<float>();
  const Eigen::Vector3f t = T.translation().cast<float>();
  std::vector<Eigen::Vector3f> out(pts.size());
  for (std::size_t k = 0; k < pts.size(); ++k) out[k] = R * pts[k] + t;
  return out;
}

double rot_angle(const Pose& T) { return so3_log(T.linear()).norm(); }

std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char* f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}

}  // namespace

Backend::Backend(const BackendParams& p) : p_(p), sc_db_(p.sc) {}
Backend::~Backend() = default;

void Backend::log(const std::string& msg) const {
  if (log_cb_) log_cb_(msg);
  else std::fprintf(stderr, "[backend] %s\n", msg.c_str());
}

void Backend::set_state(State s, const std::string& why) {
  if (s == state_) return;
  log(fmt("%s -> %s: %s", to_string(state_), to_string(s), why.c_str()));
  state_ = s;
}

BackendOutput Backend::push(double time, const Pose& T_odom_body, const std::vector<Eigen::Vector3f>& points_odom,
                            int session, const Eigen::Vector3d& up_odom) {
  const auto t0 = std::chrono::steady_clock::now();
  BackendOutput out;
  if (session != session_) {
    if (session_ >= 0) {
      // Bridge the new odometry frame assuming the robot did not move during the restart.
      T_map_odom_ = last_T_map_body_ * T_odom_body.inverse();
      if (state_ != State::kInitializing) set_state(State::kLost, "front end restarted");
    }
    session_ = session;
    waiting_reset_ = false;
    first_kf_pending_ = true;
    recent_.clear();
    last_time_ = -1.0;
  }

  if (!waiting_reset_ && last_time_ > 0 && time > last_time_) {
    const double v = (T_odom_body.translation() - last_T_odom_body_.translation()).norm() / (time - last_time_);
    if (v > p_.max_speed) {
      set_state(State::kLost, fmt("odometry speed %.1f m/s, front end diverged", v));
      waiting_reset_ = true;
      ++stats_.frontend_resets;
      if (reset_cb_) reset_cb_();
    }
  }
  last_time_ = time;
  last_T_odom_body_ = T_odom_body;

  if (!waiting_reset_) {
    recent_.push_back(points_odom);
    while (static_cast<int>(recent_.size()) > p_.kf_accum_frames) recent_.erase(recent_.begin());
    // A single scan is too sparse to relocalize on, so the first keyframe of a session merges
    // kf_accum_frames scans.
    const bool first_ready = first_kf_pending_ && static_cast<int>(recent_.size()) >= p_.kf_accum_frames;
    if (first_ready || (!first_kf_pending_ && is_keyframe(time, T_odom_body, session))) {
      first_kf_pending_ = false;
      make_keyframe(time, T_odom_body, session, up_odom);
      out.keyframe = true;
    }
    if (!kfs_.empty() && kfs_.back()->session == session)
      frames_.push_back({time, kfs_.back()->id, kfs_.back()->T_odom.inverse() * T_odom_body});
  }

  out.T_map_odom = T_map_odom_;
  out.T_map_body = T_map_odom_ * T_odom_body;
  out.state = state_;
  out.trusted = state_ == State::kNormal || state_ == State::kObsDegraded;
  if (out.trusted) { last_T_map_body_ = out.T_map_body; ever_trusted_ = true; }
  else if (!ever_trusted_) last_T_map_body_ = out.T_map_body;
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  stats_.total_ms += ms;
  stats_.max_ms = std::max(stats_.max_ms, ms);
  return out;
}

bool Backend::is_keyframe(double time, const Pose& T_odom_body, int session) const {
  if (kfs_.empty() || kfs_.back()->session != session) return true;
  const Keyframe& last = *kfs_.back();
  const Pose d = last.T_odom.inverse() * T_odom_body;
  const double dist = d.translation().norm();
  if (dist >= p_.kf_dist) return true;
  if (rot_angle(d) >= p_.kf_angle_deg * M_PI / 180.0) return true;
  return dist > 0.1 && time - last.time >= p_.kf_time;
}

void Backend::make_keyframe(double time, const Pose& T_odom_body, int session, const Eigen::Vector3d& up_odom) {
  auto kf = std::make_unique<Keyframe>();
  kf->id = static_cast<int>(kfs_.size());
  kf->time = time;
  kf->session = session;
  kf->T_odom = T_odom_body;
  kf->T_map = T_map_odom_ * T_odom_body;
  const Pose body_from_odom = T_odom_body.inverse();
  std::vector<Eigen::Vector3f> merged;
  for (const auto& scan : recent_) {
    const std::vector<Eigen::Vector3f> b = transform(scan, body_from_odom);
    merged.insert(merged.end(), b.begin(), b.end());
  }
  kf->cloud = voxel_filter(merged, p_.kf_voxel);
  kf->up_body = (T_odom_body.linear().transpose() * up_odom).normalized();
  Pose level = Pose::Identity();
  level.linear() = leveling(kf->up_body);
  kf->sc = ScanContext::make(transform(kf->cloud, level), p_.sc);
  const bool new_session = kfs_.empty() || kfs_.back()->session != session;
  kfs_.push_back(std::move(kf));
  ++stats_.keyframes;
  Keyframe& cur = *kfs_.back();
  if (p_.mode == Mode::kMapping) handle_mapping(cur, new_session);
  else handle_localization(cur, new_session);
  recent_.clear();
}

void Backend::add_odometry_factor(const Keyframe& prev, const Keyframe& cur) {
  const Pose z = prev.T_odom.inverse() * cur.T_odom;
  const double d = z.translation().norm();
  const double st = p_.odom_sigma_t + p_.odom_sigma_t_per_m * d;
  const double sr = p_.odom_sigma_r + p_.odom_sigma_r_per_m * d + p_.odom_sigma_r_per_rad * rot_angle(z);
  Vec6 s;
  s << st, st, st, sr, sr, sr;
  graph_.add_between(prev.id, cur.id, z, sqrt_info_from_sigmas(s));
}

void Backend::add_gravity_factor(const Keyframe& kf) {
  if (p_.gravity_sigma > 0) graph_.add_gravity(kf.id, kf.up_body, p_.gravity_sigma);
}

std::vector<Eigen::Vector3f> Backend::registration_source(const Keyframe& kf) const {
  const std::size_t n = kf.cloud.size();
  if (static_cast<int>(n) <= p_.reg_max_points) return kf.cloud;
  std::vector<Eigen::Vector3f> out;
  out.reserve(p_.reg_max_points);
  const double step = static_cast<double>(n) / p_.reg_max_points;
  for (int k = 0; k < p_.reg_max_points; ++k) out.push_back(kf.cloud[static_cast<std::size_t>(k * step)]);
  return out;
}

// SC compares leveled frames L = leveling(up) * body. With a gravity-aligned map, R_map_Lc is a pure
// yaw; the match gives R_Lc_Lq = Rz(yaw), so R_map_q = R_map_Lc * Rz(yaw) * leveling(up_q).
Pose Backend::leveled_pose_from_sc(const Pose& T_map_c, const Eigen::Vector3d& up_c, double yaw,
                                   const Keyframe& q) const {
  const double yaw_c = yaw_of(T_map_c.linear() * leveling(up_c).transpose());
  Pose T = Pose::Identity();
  T.linear() = rot_z(yaw_c) * rot_z(yaw) * leveling(q.up_body);
  T.translation() = T_map_c.translation();
  return T;
}

void Backend::optimize_and_update() {
  if (graph_.size() == 0) return;
  graph_.optimize(10);
  for (int id : graph_.ids()) kfs_[id]->T_map = graph_.pose(id);
  const Keyframe& last = *kfs_.back();
  if (state_ != State::kSuspect && graph_.has_node(last.id)) T_map_odom_ = normalized(last.T_map * last.T_odom.inverse());
}

// ---------------------------------------------------------------- mapping

void Backend::handle_mapping(Keyframe& kf, bool new_session) {
  graph_.add_node(kf.id, kf.T_map);
  add_gravity_factor(kf);
  if (!new_session) {
    add_odometry_factor(*kfs_[kf.id - 1], kf);
  } else if (kf.id == 0) {
    graph_.add_prior(kf.id, kf.T_map, sqrt_info_from_sigmas(Vec6::Constant(1e-4)));
    set_state(State::kNormal, "mapping started");
  } else {
    // New odometry session: only a weak prior until a loop closure ties it to the old map.
    Vec6 s;
    s << 50, 50, 50, M_PI, M_PI, M_PI;
    graph_.add_prior(kf.id, kf.T_map, sqrt_info_from_sigmas(s));
  }
  sc_db_.add(kf.sc);
  if (p_.loop_enabled && kf.id - last_loop_kf_ > p_.loop_cooldown) try_loop(kf);
  optimize_and_update();
}

bool Backend::try_loop(Keyframe& kf) {
  auto old_enough = [&](int j) { return kf.time - kfs_[j]->time >= p_.loop_min_time_gap; };
  std::vector<std::pair<int, Pose>> cands;
  int prox = -1;
  double best = p_.loop_radius;
  for (int j = 0; j < kf.id; ++j) {
    if (!old_enough(j)) continue;
    const double d = (kfs_[j]->T_map.translation() - kf.T_map.translation()).norm();
    if (d < best) { best = d; prox = j; }
  }
  if (prox >= 0) cands.emplace_back(prox, kf.T_map);
  const ScMatch m = sc_db_.query(kf.sc, [&](int j) { return j < kf.id && old_enough(j); });
  if (m.index >= 0 && m.distance < p_.loop_sc_thr && m.index != prox)
    cands.emplace_back(m.index, leveled_pose_from_sc(kfs_[m.index]->T_map, kfs_[m.index]->up_body, m.yaw, kf));

  const std::vector<Eigen::Vector3f> src = registration_source(kf);
  for (const auto& [c, init] : cands) {
    ++stats_.loops_tried;
    std::vector<Eigen::Vector3f> sub;
    for (int i = std::max(0, c - p_.loop_submap_half); i <= std::min(kf.id - 1, c + p_.loop_submap_half); ++i) {
      if (!old_enough(i)) continue;
      const std::vector<Eigen::Vector3f> w = transform(kfs_[i]->cloud, kfs_[i]->T_map);
      sub.insert(sub.end(), w.begin(), w.end());
    }
    TargetMap target(p_.reg);
    target.build(sub);
    const RegistrationResult r = register_scan(target, src, init, p_.reg, true);
    if (!r.converged || r.rank < 6 || r.inlier_ratio < p_.min_inlier_ratio || r.rmse > p_.max_rmse) continue;
    const Pose z = kfs_[c]->T_map.inverse() * r.T;
    const Innovation inn = graph_.innovation_between(c, kf.id, z, r.sqrt_info);
    if (!inn.valid || inn.nis > chi2_quantile(inn.dof, p_.loop_nis_prob)) {
      ++stats_.loops_rejected_nis;
      log(fmt("loop %d-%d rejected: NIS %.1f/%d, inliers %.2f", c, kf.id, inn.nis, inn.dof, r.inlier_ratio));
      continue;
    }
    graph_.add_between(c, kf.id, z, r.sqrt_info, p_.huber);
    ++stats_.loops_accepted;
    last_loop_kf_ = kf.id;
    const Eigen::Quaterniond zq(z.linear());
    log(fmt("loop %d-%d accepted: NIS %.1f/%d, inliers %.2f, rmse %.3f, correction %.2f m, rank %d, z %.4f %.4f %.4f "
            "%.5f %.5f %.5f %.5f, t %.6f %.6f",
            c, kf.id, inn.nis, inn.dof, r.inlier_ratio, r.rmse, (r.T.translation() - kf.T_map.translation()).norm(),
            r.rank, z.translation().x(), z.translation().y(), z.translation().z(), zq.x(), zq.y(), zq.z(), zq.w(),
            kfs_[c]->time, kf.time));
    if (state_ == State::kLost) set_state(State::kNormal, "loop closed to the existing map");
    return true;
  }
  return false;
}

// ---------------------------------------------------------------- localization

void Backend::anchor_at(Keyframe& kf, const Pose& T_map_body, const SqrtInfo& L) {
  graph_.clear();
  graph_.add_node(kf.id, T_map_body);
  graph_.add_prior(kf.id, T_map_body, L);
  kf.T_map = T_map_body;
  T_map_odom_ = normalized(T_map_body * kf.T_odom.inverse());
  conflicts_ = 0;
  reloc_failures_ = 0;
}

bool Backend::relocalize(Keyframe& kf, bool use_prediction) {
  const std::vector<Eigen::Vector3f> src = registration_source(kf);
  std::vector<Pose> inits;
  if (use_prediction) {
    const Pose base = T_map_odom_ * kf.T_odom;
    const int steps = std::max(1, static_cast<int>(std::lround(360.0 / p_.reloc_yaw_step_deg)));
    for (int k = 0; k < steps; ++k) {
      const double off = (k % 2 ? 1 : -1) * ((k + 1) / 2) * 2 * M_PI / steps;
      Pose T = base;
      T.linear() = rot_z(off) * base.linear();
      inits.push_back(T);
    }
  }
  for (const ScMatch& m : sc_db_.query_k(kf.sc, p_.reloc_sc_candidates, [](int) { return true; }))
    inits.push_back(leveled_pose_from_sc(map_kf_poses_[m.index], map_kf_up_[m.index], m.yaw, kf));
  if (inits.empty()) return false;

  RegistrationResult best, second;
  for (const Pose& init : inits) {
    const RegistrationResult r = register_scan(*prior_map_, src, init, p_.reg, true);
    if (!r.converged || r.rank < 6 || r.rmse > p_.max_rmse) continue;
    if (r.inlier_ratio > best.inlier_ratio) {
      const bool same = (r.T.translation() - best.T.translation()).norm() < 1.0 &&
                        rot_angle(best.T.inverse() * r.T) < 0.2;
      if (!same && best.converged) second = best;
      best = r;
    } else if (r.inlier_ratio > second.inlier_ratio) {
      const bool same = (r.T.translation() - best.T.translation()).norm() < 1.0 &&
                        rot_angle(best.T.inverse() * r.T) < 0.2;
      if (!same) second = r;
    }
  }
  if (!best.converged || best.inlier_ratio < p_.reloc_min_inlier_ratio) {
    log(fmt("relocalization failed at kf %d (t %.3f): best inliers %.2f over %zu hypotheses, (%.2f, %.2f, %.2f) yaw %.1f",
            kf.id, kf.time, best.inlier_ratio, inits.size(), best.T.translation().x(), best.T.translation().y(),
            best.T.translation().z(), yaw_of(best.T.linear()) * 180 / M_PI));
    return false;
  }
  if (second.converged && second.inlier_ratio > best.inlier_ratio - 0.05) {
    log(fmt("relocalization ambiguous at kf %d: %.2f vs %.2f", kf.id, best.inlier_ratio, second.inlier_ratio));
    return false;
  }
  anchor_at(kf, best.T, best.sqrt_info);
  ++stats_.relocalizations;
  log(fmt("relocalized at kf %d (t %.3f): inliers %.2f, rmse %.3f, (%.2f, %.2f, %.2f) yaw %.1f", kf.id, kf.time,
          best.inlier_ratio, best.rmse, best.T.translation().x(), best.T.translation().y(), best.T.translation().z(),
          yaw_of(best.T.linear()) * 180 / M_PI));
  return true;
}

void Backend::handle_localization(Keyframe& kf, bool new_session) {
  if (!prior_map_) {
    if (state_ == State::kInitializing && kf.id == 0) log("localization mode without a prior map");
    return;
  }
  if (state_ == State::kInitializing || state_ == State::kLost) {
    const bool use_pred = state_ == State::kLost ? ever_trusted_ : p_.has_initial_pose;
    if (state_ == State::kInitializing && p_.has_initial_pose && kf.id == 0) T_map_odom_ = p_.initial_pose;
    if (relocalize(kf, use_pred)) set_state(State::kNormal, "relocalized");
    return;
  }

  // No gravity factor here: the prior map observes roll and pitch, and the front end's gravity can be
  // off for a while after a start in motion.
  graph_.add_node(kf.id, kf.T_map);
  if (!new_session && graph_.has_node(kf.id - 1)) add_odometry_factor(*kfs_[kf.id - 1], kf);
  else graph_.add_prior(kf.id, kf.T_map, sqrt_info_from_sigmas(Vec6::Constant(1.0)));

  if (state_ == State::kSuspect) {
    if (relocalize(kf, true)) {
      set_state(State::kNormal, "relocalized");
    } else if (++reloc_failures_ >= p_.suspect_to_lost) {
      set_state(State::kLost, "relocalization keeps failing, restarting the front end");
      graph_.clear();
      waiting_reset_ = true;
      ++stats_.frontend_resets;
      if (reset_cb_) reset_cb_();
    }
    return;
  }

  // NORMAL / OBS_DEGRADED: scan-to-map factor around the prediction.
  ++stats_.map_tried;
  const RegistrationResult r = register_scan(*prior_map_, registration_source(kf), kf.T_map, p_.reg, false);
  bool conflict = false;
  if (!r.converged || r.rank == 0 || r.inlier_ratio < p_.min_inlier_ratio || r.rmse > p_.max_rmse) {
    ++stats_.map_rejected_quality;
    // So few inliers that the prediction no longer overlaps the map.
    conflict = r.inlier_ratio < 0.5 * p_.min_inlier_ratio && r.num_points > 500;
  } else {
    const Innovation inn = graph_.innovation_prior(kf.id, r.T, r.sqrt_info);
    if (inn.valid && inn.nis <= chi2_quantile(inn.dof, p_.nis_prob)) {
      graph_.add_prior(kf.id, r.T, r.sqrt_info, p_.huber);
      ++stats_.map_accepted;
      conflicts_ = 0;
      set_state(State::kNormal, "observation accepted");
    } else {
      ++stats_.map_rejected_nis;
      conflict = true;
      log(fmt("map factor at kf %d rejected: NIS %.1f/%d, inliers %.2f", kf.id, inn.nis, inn.dof, r.inlier_ratio));
    }
  }
  if (conflict) {
    ++conflicts_;
    if (conflicts_ >= p_.degraded_to_suspect) {
      set_state(State::kSuspect, fmt("%d conflicting observations", conflicts_));
      reloc_failures_ = 0;
    } else {
      set_state(State::kObsDegraded, "observation conflicts with prediction");
    }
  }
  optimize_and_update();
  while (static_cast<int>(graph_.size()) > p_.window) {
    const int oldest = graph_.ids().front();
    graph_.marginalize_oldest();
    kfs_[oldest]->cloud.clear();
    kfs_[oldest]->cloud.shrink_to_fit();
  }
}

// ---------------------------------------------------------------- map package and results

bool Backend::save_map(const std::string& dir, std::string* error) const {
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories(dir, ec);
  std::ofstream kf_file(fs::path(dir) / "keyframes.txt");
  std::ofstream cloud_file(fs::path(dir) / "clouds.bin", std::ios::binary);
  if (!kf_file || !cloud_file) { if (error) *error = "cannot write " + dir; return false; }
  kf_file << "# id time session x y z qx qy qz qw num_points (body cloud in clouds.bin, float32 xyz)\n"
          << std::fixed << std::setprecision(9);
  int saved = 0;
  for (const auto& kf : kfs_) {
    if (kf->cloud.empty()) continue;
    const Eigen::Quaterniond q(kf->T_map.linear());
    const Eigen::Vector3d t = kf->T_map.translation();
    kf_file << saved << ' ' << kf->time << ' ' << kf->session << ' ' << t.x() << ' ' << t.y() << ' ' << t.z() << ' '
            << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << ' ' << kf->cloud.size() << '\n';
    const uint32_t n = static_cast<uint32_t>(kf->cloud.size());
    cloud_file.write(reinterpret_cast<const char*>(&n), sizeof(n));
    for (const auto& p : kf->cloud) cloud_file.write(reinterpret_cast<const char*>(p.data()), 3 * sizeof(float));
    ++saved;
  }
  if (!zhao_lio::write_pcd((fs::path(dir) / "map.pcd").string(), map_cloud(0.1f))) {
    if (error) *error = "cannot write map.pcd";
    return false;
  }
  return static_cast<bool>(kf_file) && static_cast<bool>(cloud_file);
}

bool Backend::load_map(const std::string& path, std::string* error) {
  namespace fs = std::filesystem;
  std::vector<Eigen::Vector3f> pts;
  map_kf_poses_.clear();
  map_kf_up_.clear();
  sc_db_.clear();
  if (fs::is_directory(path)) {
    std::ifstream kf_file(fs::path(path) / "keyframes.txt");
    std::ifstream cloud_file(fs::path(path) / "clouds.bin", std::ios::binary);
    if (!kf_file || !cloud_file) { if (error) *error = path + ": missing keyframes.txt or clouds.bin"; return false; }
    std::string line;
    while (std::getline(kf_file, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream ss(line);
      int id, session;
      double time, x, y, z, qx, qy, qz, qw;
      std::size_t n;
      if (!(ss >> id >> time >> session >> x >> y >> z >> qx >> qy >> qz >> qw >> n)) continue;
      uint32_t stored = 0;
      cloud_file.read(reinterpret_cast<char*>(&stored), sizeof(stored));
      std::vector<Eigen::Vector3f> cloud(stored);
      cloud_file.read(reinterpret_cast<char*>(cloud.data()), static_cast<std::streamsize>(stored * 3 * sizeof(float)));
      if (!cloud_file || stored != n) { if (error) *error = path + ": clouds.bin does not match keyframes.txt"; return false; }
      const Pose T = pose_from(Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix(), {x, y, z});
      // The map frame is gravity aligned (mapping keeps keyframes on the front end's gravity).
      const Eigen::Vector3d up = T.linear().transpose() * Eigen::Vector3d::UnitZ();
      map_kf_poses_.push_back(T);
      map_kf_up_.push_back(up);
      Pose level = Pose::Identity();
      level.linear() = leveling(up);
      sc_db_.add(ScanContext::make(transform(cloud, level), p_.sc));
      const std::vector<Eigen::Vector3f> w = transform(cloud, T);
      pts.insert(pts.end(), w.begin(), w.end());
    }
  } else if (!read_pcd(path, pts, error)) {
    return false;
  }
  if (pts.empty()) { if (error) *error = path + ": empty map"; return false; }
  prior_map_ = std::make_unique<TargetMap>(p_.reg);
  prior_map_->build(pts);
  log(fmt("prior map: %zu points, %zu relocalization keyframes", pts.size(), sc_db_.size()));
  return true;
}

std::vector<Backend::TimedPose> Backend::optimized_trajectory() const {
  std::vector<TimedPose> out;
  out.reserve(frames_.size());
  for (const Frame& f : frames_) out.push_back({f.time, kfs_[f.kf]->T_map * f.T_kf_body});
  return out;
}

std::vector<Backend::TimedPose> Backend::keyframe_poses() const {
  std::vector<TimedPose> out;
  for (const auto& kf : kfs_) out.push_back({kf->time, kf->T_map});
  return out;
}

std::vector<Eigen::Vector3f> Backend::map_cloud(float voxel) const {
  std::vector<Eigen::Vector3f> all;
  for (const auto& kf : kfs_) {
    const std::vector<Eigen::Vector3f> w = transform(kf->cloud, kf->T_map);
    all.insert(all.end(), w.begin(), w.end());
  }
  return voxel > 0 ? voxel_filter(all, voxel) : all;
}

}  // namespace zhao_lio_backend
