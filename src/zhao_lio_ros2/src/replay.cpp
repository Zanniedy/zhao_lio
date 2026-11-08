#include <cstdio>
#include <fstream>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include "zhao_lio/zhao_lio.h"
#include "zhao_lio/io.h"
#include "zhao_lio/sync.h"
#include "zhao_lio_ros2/backend_config.h"
#include "zhao_lio_ros2/config.h"
#include "zhao_lio_ros2/map_saver.h"
#include "zhao_lio_ros2/msg_conversions.h"

// Reads a bag directly and feeds zhao_lio in timestamp order, without ROS transport, so per-frame
// timing does not depend on playback rate.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("zhao_lio");
  const zhao_lio_ros2::NodeConfig cfg = zhao_lio_ros2::load_config(*node);
  const std::string bag = node->declare_parameter<std::string>("bag", "");
  const double start_offset = node->declare_parameter<double>("start_offset", 0.0);  // s skipped at the bag start
  const zhao_lio_ros2::BackendNodeConfig bcfg = zhao_lio_ros2::load_backend_config(*node);
  // Test hook: from test_perturb_time on, the odometry fed to the back end is offset by
  // test_perturb = [dx, dy, yaw_deg] (simulates a front-end jump the back end must detect).
  const double perturb_time = node->declare_parameter<double>("test_perturb_time", -1.0);
  const auto perturb = node->declare_parameter<std::vector<double>>("test_perturb", {0.0, 0.0, 0.0});
  const double perturb_ramp = node->declare_parameter<double>("test_perturb_ramp", 0.0);  // s; 0 = step
  if (bag.empty()) { std::fprintf(stderr, "set -p bag:=<path>\n"); return 1; }

  rosbag2_cpp::Reader reader;
  try {
    reader.open(bag);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "cannot open bag %s: %s\n", bag.c_str(), e.what());
    return 1;
  }
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_ser;
#ifdef HAVE_LIVOX_DRIVER
  rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> livox_ser;
#else
  if (cfg.lidar_type == "livox_custom") { std::fprintf(stderr, "built without livox_ros_driver2\n"); return 1; }
#endif

  zhao_lio::ZhaoLio lio(cfg.params);
  std::unique_ptr<zhao_lio_backend::Backend> backend;
  std::ofstream btraj;
  bool reset_requested = false;
  if (bcfg.enabled) {
    backend = std::make_unique<zhao_lio_backend::Backend>(bcfg.params);
    backend->set_reset_callback([&] { reset_requested = true; });
    if (!bcfg.map_path.empty()) {
      std::string err;
      if (!backend->load_map(bcfg.map_path, &err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
    }
    if (!bcfg.traj_path.empty()) { btraj.open(bcfg.traj_path); btraj << std::fixed << std::setprecision(9); }
  }
  auto perturb_at = [&](double t) {
    const double a = perturb_ramp > 0 ? std::min(1.0, (t - perturb_time) / perturb_ramp) : 1.0;
    return zhao_lio_backend::pose_from(zhao_lio_backend::rot_z(a * perturb.at(2) * M_PI / 180.0),
                                       Eigen::Vector3d(a * perturb.at(0), a * perturb.at(1), 0.0));
  };
  double bag_start = -1.0;
  long trusted_frames = 0;
  zhao_lio::MeasureSync sync;
  zhao_lio::TumWriter tum;
  zhao_lio::TimingCsv csv;
  if (!cfg.traj_path.empty()) tum.open(cfg.traj_path);
  if (!cfg.timing_csv.empty()) csv.open(cfg.timing_csv);

  zhao_lio::FrameTiming sum;
  long frames = 0, used = 0, rematched = 0, down = 0;
  zhao_lio::Measure m;
  zhao_lio::ScanResult r;
  while (reader.has_next()) {
    auto msg = reader.read_next();
    const double stamp = msg->time_stamp * 1e-9;
    if (bag_start < 0) bag_start = stamp;
    if (stamp - bag_start < start_offset) continue;
    rclcpp::SerializedMessage ser(*msg->serialized_data);
    if (msg->topic_name == cfg.imu_topic) {
      sensor_msgs::msg::Imu imu;
      imu_ser.deserialize_message(&ser, &imu);
      sync.push_imu(zhao_lio_ros2::imu_from_msg(imu, cfg.acc_scale));
    } else if (msg->topic_name == cfg.lidar_topic) {
      zhao_lio::ScanFrame f;
      if (cfg.lidar_type == "pointcloud2") {
        sensor_msgs::msg::PointCloud2 pc;
        pc2_ser.deserialize_message(&ser, &pc);
        if (!zhao_lio_ros2::scan_from_pointcloud2(pc, f)) { std::fprintf(stderr, "no 'timestamp' field\n"); return 1; }
      } else {
#ifdef HAVE_LIVOX_DRIVER
        livox_ros_driver2::msg::CustomMsg lm;
        livox_ser.deserialize_message(&ser, &lm);
        f = zhao_lio_ros2::scan_from_livox(lm);
#endif
      }
      sync.push_scan(std::move(f));
    } else {
      continue;
    }
    while (sync.pop(m)) {
      if (!lio.process(m, r)) continue;
      tum.write(r.time, r.state);
      if (backend) {
        zhao_lio_backend::Pose T = zhao_lio_backend::pose_from(r.state.rot, r.state.pos);
        if (perturb_time >= 0 && r.time - bag_start >= perturb_time) {
          const zhao_lio_backend::Pose perturb_T = perturb_at(r.time - bag_start);
          T = perturb_T * T;
          for (auto& p : r.points_world) p = (perturb_T * p.cast<double>()).cast<float>();
        }
        const zhao_lio_backend::BackendOutput bo = backend->push(r.time, T, r.points_world, lio.session(),
                                                                    -(T.linear() * r.state.rot.transpose() * r.state.grav).normalized());
        if (bo.trusted) ++trusted_frames;
        if (btraj.is_open() && bo.trusted) zhao_lio_ros2::write_tum(btraj, r.time, bo.T_map_body);
        if (reset_requested) { lio.reset(); reset_requested = false; }
      }
      csv.write(r);
      ++frames;
      used += r.stats.points_used;
      rematched += r.stats.rematched;
      down += static_cast<long>(r.num_downsampled);
      sum.preprocess_ms += r.timing.preprocess_ms; sum.downsample_ms += r.timing.downsample_ms;
      sum.prematch_ms += r.timing.prematch_ms; sum.update_ms += r.timing.update_ms;
      sum.map_ms += r.timing.map_ms; sum.total_ms += r.timing.total_ms;
    }
  }

  if (frames == 0) { std::fprintf(stderr, "no frames processed (check topics and lidar_type)\n"); return 1; }
  const double n = static_cast<double>(frames);
  std::printf("frames %ld, dropped scans %zu, mean downsampled %.0f, mean used %.0f, rematch ratio %.4f\n",
              frames, sync.dropped_scans(), down / n, used / n, used ? static_cast<double>(rematched) / used : 0.0);
  std::printf("mean ms: pre %.3f  ds %.3f  prematch %.3f  update %.3f  map %.3f  total %.3f\n",
              sum.preprocess_ms / n, sum.downsample_ms / n, sum.prematch_ms / n, sum.update_ms / n,
              sum.map_ms / n, sum.total_ms / n);
  if (backend) {
    const auto& st = backend->stats();
    std::printf("backend: state %s, trusted frames %ld/%ld, keyframes %d, loops %d/%d (NIS rejected %d), "
                "map factors %d/%d (NIS rejected %d, poor match %d), relocalizations %d, front-end resets %d, "
                "mean %.2f ms, max %.1f ms\n",
                zhao_lio_backend::to_string(backend->state()), trusted_frames, frames, st.keyframes, st.loops_accepted,
                st.loops_tried, st.loops_rejected_nis, st.map_accepted, st.map_tried, st.map_rejected_nis,
                st.map_rejected_quality, st.relocalizations, st.frontend_resets, st.total_ms / frames, st.max_ms);
    if (!bcfg.opt_traj_path.empty()) {
      std::ofstream f(bcfg.opt_traj_path);
      f << std::fixed << std::setprecision(9);
      for (const auto& tp : backend->optimized_trajectory()) zhao_lio_ros2::write_tum(f, tp.time, tp.T);
    }
    if (!bcfg.save_dir.empty() && bcfg.params.mode == zhao_lio_backend::Mode::kMapping) {
      std::string err;
      if (backend->save_map(bcfg.save_dir, &err)) std::printf("map package -> %s\n", bcfg.save_dir.c_str());
      else std::fprintf(stderr, "map package not saved: %s\n", err.c_str());
    }
  }
  zhao_lio_ros2::MapSaver saver(cfg.map_save_dir);
  if (saver.enabled()) {
    const long pts = saver.save(lio);
    if (pts >= 0) std::printf("map: %ld points -> %s\n", pts, saver.path().c_str());
    else std::fprintf(stderr, "map not saved: %s\n", saver.error().c_str());
  }
  rclcpp::shutdown();
  return 0;
}
