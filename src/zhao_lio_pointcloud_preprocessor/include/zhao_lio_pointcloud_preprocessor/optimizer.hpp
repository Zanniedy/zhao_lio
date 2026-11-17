#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace zhao_lio_pointcloud_preprocessor {

struct OptimizerConfig {
  double min_range = 0.3;
  double max_range = 100.0;
  double voxel_size = 0.10;
  bool height_filter_enabled = false;
  double min_z = -0.10;
  double min_intensity = 0.0;
  double max_z = 4.0;
};

struct OptimizerStats {
  std::size_t low_height = 0;
  std::size_t low_intensity = 0;
  std::size_t high_height = 0;
  bool height_applied = false;
  bool intensity_applied = false;
};

class PointCloudOptimizer {
 public:
  explicit PointCloudOptimizer(OptimizerConfig config = {});

  sensor_msgs::msg::PointCloud2 optimize(
      const sensor_msgs::msg::PointCloud2& input,
      const std::optional<Eigen::Isometry3d>& cloud_to_height = std::nullopt,
      OptimizerStats* stats = nullptr) const;

 private:
  struct VoxelKey {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelHash {
    std::size_t operator()(const VoxelKey& key) const;
  };

  struct FieldInfo {
    std::uint32_t offset = 0;
    std::uint8_t datatype = 0;
    bool present = false;
  };

  static FieldInfo find_field(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name);
  static bool read_scalar(const std::uint8_t* data, const FieldInfo& field, double& value);

  OptimizerConfig config_;
};

}  // namespace zhao_lio_pointcloud_preprocessor
