#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include "zhao_lio/zhao_lio.h"
#include "zhao_lio/io.h"
#include "zhao_lio/sync.h"
#include "zhao_lio_ros2/config.h"
#include "zhao_lio_ros2/msg_conversions.h"

// Reads a bag directly and feeds zhao_lio in timestamp order, without ROS transport, so per-frame
// timing does not depend on playback rate.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("zhao_lio");
  const zhao_lio_ros2::NodeConfig cfg = zhao_lio_ros2::load_config(*node);
  const std::string bag = node->declare_parameter<std::string>("bag", "");
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
  rclcpp::shutdown();
  return 0;
}
