#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "zhao_lio_backend/backend.h"
#include "zhao_lio_ros2/config.h"

namespace zhao_lio_ros2 {

struct BackendNodeConfig {
  bool enabled = false;
  zhao_lio_backend::BackendParams params;
  std::string map_path;   // prior map (localization): map package directory or PCD
  std::string save_dir;   // map package written here on exit, mapping mode only; empty disables
  std::string traj_path;  // real-time map-frame TUM
  std::string opt_traj_path;  // every frame re-expressed with the final keyframe estimates
  std::string map_frame;
};

// Parameters live under "backend.*". backend.mode: "" (off) | mapping | localization.
inline BackendNodeConfig load_backend_config(rclcpp::Node& n) {
  BackendNodeConfig c;
  auto& p = c.params;
  auto d = [&](const std::string& name, double v) { return n.declare_parameter<double>("backend." + name, v); };
  auto i = [&](const std::string& name, int v) {
    return static_cast<int>(n.declare_parameter<int64_t>("backend." + name, v));
  };
  const std::string mode = n.declare_parameter<std::string>("backend.mode", "");
  c.enabled = !mode.empty();
  if (c.enabled) p.mode = zhao_lio_backend::parse_mode(mode);
  c.map_path = expand_home(n.declare_parameter<std::string>("backend.map_path", ""));
  c.save_dir = expand_home(n.declare_parameter<std::string>("backend.save_dir", ""));
  c.traj_path = expand_home(n.declare_parameter<std::string>("backend.traj_path", ""));
  c.opt_traj_path = expand_home(n.declare_parameter<std::string>("backend.opt_traj_path", ""));
  c.map_frame = n.declare_parameter<std::string>("backend.map_frame", "map");

  p.kf_dist = d("kf_dist", p.kf_dist);
  p.kf_angle_deg = d("kf_angle_deg", p.kf_angle_deg);
  p.kf_time = d("kf_time", p.kf_time);
  p.kf_accum_frames = i("kf_accum_frames", p.kf_accum_frames);
  p.kf_voxel = static_cast<float>(d("kf_voxel", p.kf_voxel));
  p.reg_max_points = i("reg_max_points", p.reg_max_points);
  p.odom_sigma_t = d("odom_sigma_t", p.odom_sigma_t);
  p.odom_sigma_t_per_m = d("odom_sigma_t_per_m", p.odom_sigma_t_per_m);
  p.odom_sigma_r = d("odom_sigma_r", p.odom_sigma_r);
  p.odom_sigma_r_per_m = d("odom_sigma_r_per_m", p.odom_sigma_r_per_m);
  p.odom_sigma_r_per_rad = d("odom_sigma_r_per_rad", p.odom_sigma_r_per_rad);
  p.gravity_sigma = d("gravity_sigma", p.gravity_sigma);
  p.max_speed = d("max_speed", p.max_speed);
  p.nis_prob = d("nis_prob", p.nis_prob);
  p.huber = d("huber", p.huber);
  p.min_inlier_ratio = d("min_inlier_ratio", p.min_inlier_ratio);
  p.max_rmse = d("max_rmse", p.max_rmse);

  p.reg.coarse_resolution = static_cast<float>(d("reg.coarse_resolution", p.reg.coarse_resolution));
  p.reg.fine_resolution = static_cast<float>(d("reg.fine_resolution", p.reg.fine_resolution));
  p.reg.coarse_plane_thr = d("reg.coarse_plane_thr", p.reg.coarse_plane_thr);
  p.reg.fine_plane_thr = d("reg.fine_plane_thr", p.reg.fine_plane_thr);
  p.reg.max_iters = i("reg.max_iters", p.reg.max_iters);
  p.reg.huber = d("reg.huber", p.reg.huber);
  p.reg.inlier_thr = d("reg.inlier_thr", p.reg.inlier_thr);
  p.reg.sigma_floor = d("reg.sigma_floor", p.reg.sigma_floor);
  p.reg.n_eff = d("reg.n_eff", p.reg.n_eff);
  p.reg.rot_scale = d("reg.rot_scale", p.reg.rot_scale);
  p.reg.max_sigma = d("reg.max_sigma", p.reg.max_sigma);
  p.reg.min_eig_ratio = d("reg.min_eig_ratio", p.reg.min_eig_ratio);
  p.reg.sigma_min = d("reg.sigma_min", p.reg.sigma_min);

  p.sc.rings = i("sc.rings", p.sc.rings);
  p.sc.sectors = i("sc.sectors", p.sc.sectors);
  p.sc.max_range = d("sc.max_range", p.sc.max_range);
  p.sc.height_offset = d("sc.height_offset", p.sc.height_offset);

  p.loop_enabled = n.declare_parameter<bool>("backend.loop_enabled", p.loop_enabled);
  p.loop_min_time_gap = d("loop_min_time_gap", p.loop_min_time_gap);
  p.loop_radius = d("loop_radius", p.loop_radius);
  p.loop_submap_half = i("loop_submap_half", p.loop_submap_half);
  p.loop_sc_thr = d("loop_sc_thr", p.loop_sc_thr);
  p.loop_nis_prob = d("loop_nis_prob", p.loop_nis_prob);
  p.loop_cooldown = i("loop_cooldown", p.loop_cooldown);

  p.window = i("window", p.window);
  p.degraded_to_suspect = i("degraded_to_suspect", p.degraded_to_suspect);
  p.suspect_to_lost = i("suspect_to_lost", p.suspect_to_lost);
  p.reloc_min_inlier_ratio = d("reloc_min_inlier_ratio", p.reloc_min_inlier_ratio);
  p.reloc_sc_candidates = i("reloc_sc_candidates", p.reloc_sc_candidates);
  p.reloc_yaw_step_deg = d("reloc_yaw_step_deg", p.reloc_yaw_step_deg);
  // [x, y, z, yaw_deg] of the odometry origin in the map; empty = unknown (global relocalization).
  const auto init = n.declare_parameter<std::vector<double>>("backend.initial_pose", std::vector<double>{});
  if (init.size() == 4) {
    p.has_initial_pose = true;
    p.initial_pose = zhao_lio_backend::pose_from(zhao_lio_backend::rot_z(init[3] * M_PI / 180.0),
                                                 Eigen::Vector3d(init[0], init[1], init[2]));
  } else if (!init.empty()) {
    throw std::invalid_argument("backend.initial_pose needs [x, y, z, yaw_deg]");
  }
  return c;
}

inline void write_tum(std::ostream& f, double t, const zhao_lio_backend::Pose& T) {
  const Eigen::Quaterniond q(T.linear());
  const Eigen::Vector3d p = T.translation();
  f << t << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' '
    << q.w() << '\n';
}

}  // namespace zhao_lio_ros2
