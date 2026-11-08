#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "zhao_lio_backend/pose_graph.h"
#include "zhao_lio_backend/registration.h"
#include "zhao_lio_backend/scan_context.h"
#include "zhao_lio_backend/se3.h"

namespace zhao_lio_backend {

enum class Mode { kMapping, kLocalization };
enum class State { kInitializing, kNormal, kObsDegraded, kSuspect, kLost };
const char* to_string(State s);
Mode parse_mode(const std::string& s);

struct BackendParams {
  Mode mode = Mode::kMapping;

  // Keyframe selection and storage.
  double kf_dist = 1.0;          // m
  double kf_angle_deg = 10.0;
  double kf_time = 2.0;          // s, only while moving (dist > 0.1 m)
  int kf_accum_frames = 10;      // recent scans merged into one keyframe cloud
  float kf_voxel = 0.25f;        // m, keyframe cloud resolution
  int reg_max_points = 4000;     // registration source size (uniform subsample)

  // Odometry factor noise: sigma = base + per_m * distance (+ per_rad * angle for rotation).
  double odom_sigma_t = 0.02, odom_sigma_t_per_m = 0.02;
  double odom_sigma_r = 0.002, odom_sigma_r_per_m = 0.002, odom_sigma_r_per_rad = 0.02;
  double gravity_sigma = 0.003;  // rad; mapping mode: keyframe roll/pitch follow the front end's gravity
  double max_speed = 15.0;       // m/s; faster odometry means the front end diverged

  // Observation gating.
  double nis_prob = 0.99;        // chi-square acceptance probability for scan-to-map factors
  double huber = 3.0;            // on the whitened residual norm
  double min_inlier_ratio = 0.3;  // of the source points; narrow-FOV scans overlap the map only partly
  double max_rmse = 0.12;        // m

  RegistrationParams reg;
  ScanContextParams sc;

  // Loop closure (mapping mode).
  bool loop_enabled = true;
  double loop_min_time_gap = 30.0;  // s
  double loop_radius = 6.0;         // m, proximity search around the current estimate
  int loop_submap_half = 6;         // keyframes on each side of the candidate
  double loop_sc_thr = 0.3;
  double loop_nis_prob = 0.9999;
  int loop_cooldown = 2;            // keyframes skipped after an accepted loop

  // Localization mode.
  int window = 100;                 // keyframes kept in the graph
  bool has_initial_pose = false;
  Pose initial_pose = Pose::Identity();  // T_map_odom at start
  int degraded_to_suspect = 3;      // consecutive conflicting observations
  int suspect_to_lost = 5;          // failed relocalizations
  double reloc_min_inlier_ratio = 0.35;
  int reloc_sc_candidates = 5;
  double reloc_yaw_step_deg = 30.0; // local yaw sweep around the prediction
};

struct BackendOutput {
  Pose T_map_odom = Pose::Identity();
  Pose T_map_body = Pose::Identity();
  State state = State::kInitializing;
  bool trusted = false;
  bool keyframe = false;
};

struct BackendStats {
  int keyframes = 0;
  int loops_tried = 0, loops_accepted = 0, loops_rejected_nis = 0;
  int map_tried = 0, map_accepted = 0, map_rejected_nis = 0, map_rejected_quality = 0;
  int relocalizations = 0, frontend_resets = 0;
  double total_ms = 0.0, max_ms = 0.0;
};

class Backend {
 public:
  explicit Backend(const BackendParams& p);
  ~Backend();

  // Map package written by save_map, or a plain PCD (no relocalization database).
  bool load_map(const std::string& path, std::string* error);
  bool save_map(const std::string& dir, std::string* error) const;

  // One odometry result: body pose in the odometry frame and the scan in the odometry frame.
  // `session` must change whenever the front end restarts its odometry frame. `up_odom` is the
  // front end's current estimate of the up direction (-gravity) in the odometry frame.
  BackendOutput push(double time, const Pose& T_odom_body, const std::vector<Eigen::Vector3f>& points_odom,
                     int session, const Eigen::Vector3d& up_odom = Eigen::Vector3d::UnitZ());

  // Called when the back end decides the front end must be restarted.
  void set_reset_callback(std::function<void()> cb) { reset_cb_ = std::move(cb); }
  void set_log_callback(std::function<void(const std::string&)> cb) { log_cb_ = std::move(cb); }

  State state() const { return state_; }
  const BackendStats& stats() const { return stats_; }

  // Every pushed frame re-expressed with the final keyframe estimates (for offline evaluation).
  struct TimedPose { double time; Pose T; };
  std::vector<TimedPose> optimized_trajectory() const;
  std::vector<TimedPose> keyframe_poses() const;
  // Keyframe clouds in the map frame, voxel filtered.
  std::vector<Eigen::Vector3f> map_cloud(float voxel) const;

 private:
  struct Keyframe;
  struct Frame { double time; int kf; Pose T_kf_body; };

  bool is_keyframe(double time, const Pose& T_odom_body, int session) const;
  void make_keyframe(double time, const Pose& T_odom_body, int session, const Eigen::Vector3d& up_odom);
  void add_odometry_factor(const Keyframe& prev, const Keyframe& cur);
  void add_gravity_factor(const Keyframe& kf);
  void handle_mapping(Keyframe& kf, bool new_session);
  void handle_localization(Keyframe& kf, bool new_session);
  bool try_loop(Keyframe& kf);
  bool relocalize(Keyframe& kf, bool use_prediction);
  void anchor_at(Keyframe& kf, const Pose& T_map_body, const SqrtInfo& L);
  void optimize_and_update();
  void set_state(State s, const std::string& why);
  void log(const std::string& msg) const;
  std::vector<Eigen::Vector3f> registration_source(const Keyframe& kf) const;
  Pose leveled_pose_from_sc(const Pose& T_map_c, const Eigen::Vector3d& up_c, double yaw, const Keyframe& q) const;

  BackendParams p_;
  State state_ = State::kInitializing;
  PoseGraph graph_;
  std::vector<std::unique_ptr<Keyframe>> kfs_;
  std::vector<Frame> frames_;
  ScanContextDb sc_db_;         // mapping: own keyframes; localization: prior map keyframes
  std::vector<Pose> map_kf_poses_;  // localization: pose of each prior-map keyframe
  std::vector<Eigen::Vector3d> map_kf_up_;  // ... and its up direction in the body frame
  std::unique_ptr<TargetMap> prior_map_;
  std::vector<std::vector<Eigen::Vector3f>> recent_;  // recent scans in the odom frame
  Pose T_map_odom_ = Pose::Identity();
  int session_ = -1;
  int last_loop_kf_ = -1000;
  int conflicts_ = 0, reloc_failures_ = 0;
  double last_time_ = -1.0;
  Pose last_T_odom_body_ = Pose::Identity();
  Pose last_T_map_body_ = Pose::Identity();
  bool ever_trusted_ = false;
  bool waiting_reset_ = false;
  bool first_kf_pending_ = false;  // a new session's first keyframe waits for kf_accum_frames scans  // front-end reset requested; frames of the old session are ignored
  BackendStats stats_;
  std::function<void()> reset_cb_;
  std::function<void(const std::string&)> log_cb_;
};

}  // namespace zhao_lio_backend
