#pragma once

#include <cstdio>
#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

#include "zhao_lio/io.h"
#include "zhao_lio/zhao_lio.h"

namespace zhao_lio_ros2 {

// Writes the matching voxel map as a PCD in the odometry frame (gravity aligned: z up, origin at
// the IMU position at initialization). One file per run, named after the start time; every save
// overwrites it through a temporary file, so a crash never leaves a half-written map.
class MapSaver {
 public:
  explicit MapSaver(const std::string& dir) {
    if (dir.empty()) return;
    char name[64];
    const std::time_t now = std::time(nullptr);
    std::strftime(name, sizeof(name), "map_%Y%m%d_%H%M%S.pcd", std::localtime(&now));
    path_ = (std::filesystem::path(dir) / name).string();
  }

  bool enabled() const { return !path_.empty(); }
  const std::string& path() const { return path_; }

  // Returns the number of points written, or -1 on failure (message in error()).
  long save(const zhao_lio::ZhaoLio& lio) {
    if (!enabled()) { error_ = "map saving disabled (map_save_dir is empty)"; return -1; }
    if (lio.status() != zhao_lio::ZhaoLio::Status::kProcess) { error_ = "not initialized yet, no map"; return -1; }
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(path_).parent_path(), ec);
    lio.map().get_points(points_);
    const std::string tmp = path_ + ".tmp";
    if (!zhao_lio::write_pcd(tmp, points_) || std::rename(tmp.c_str(), path_.c_str()) != 0) {
      error_ = "cannot write " + path_;
      std::remove(tmp.c_str());
      return -1;
    }
    return static_cast<long>(points_.size());
  }

  const std::string& error() const { return error_; }

 private:
  std::string path_, error_;
  std::vector<Eigen::Vector3f> points_;
};

}  // namespace zhao_lio_ros2
