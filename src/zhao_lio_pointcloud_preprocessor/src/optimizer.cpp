#include "zhao_lio_pointcloud_preprocessor/optimizer.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <sensor_msgs/msg/point_field.hpp>

namespace zhao_lio_pointcloud_preprocessor {

namespace {

constexpr std::uint8_t kFloat32 = sensor_msgs::msg::PointField::FLOAT32;
constexpr std::uint8_t kFloat64 = sensor_msgs::msg::PointField::FLOAT64;
constexpr double kHeightTolerance = 1e-6;

std::size_t scalar_size(std::uint8_t datatype) {
  using Field = sensor_msgs::msg::PointField;
  switch (datatype) {
    case Field::INT8: case Field::UINT8: return 1;
    case Field::INT16: case Field::UINT16: return 2;
    case Field::INT32: case Field::UINT32: case Field::FLOAT32: return 4;
    case Field::FLOAT64: return 8;
    default: return 0;
  }
}

template <typename Scalar>
double load_scalar(const std::uint8_t* data) {
  Scalar value;
  std::memcpy(&value, data, sizeof(value));
  return static_cast<double>(value);
}

}  // namespace

PointCloudOptimizer::PointCloudOptimizer(OptimizerConfig config) : config_(config) {
  if (!std::isfinite(config_.min_range) || !std::isfinite(config_.max_range) ||
      config_.min_range < 0.0 || config_.max_range <= config_.min_range) {
    throw std::invalid_argument("point cloud range must satisfy 0 <= min_range < max_range");
  }
  if (!std::isfinite(config_.voxel_size) || !std::isfinite(config_.min_z) ||
      !std::isfinite(config_.max_z) || config_.min_z >= config_.max_z ||
      !std::isfinite(config_.min_intensity) || config_.min_intensity < 0.0) {
    throw std::invalid_argument("filter thresholds must be finite and min_intensity >= 0");
  }
}

std::size_t PointCloudOptimizer::VoxelHash::operator()(const VoxelKey& key) const {
  const auto mix = [](std::uint64_t value) {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
  };
  const std::uint64_t hx = mix(static_cast<std::uint64_t>(key.x));
  const std::uint64_t hy = mix(static_cast<std::uint64_t>(key.y));
  const std::uint64_t hz = mix(static_cast<std::uint64_t>(key.z));
  return static_cast<std::size_t>(hx ^ (hy << 1U) ^ (hz >> 1U));
}

PointCloudOptimizer::FieldInfo PointCloudOptimizer::find_field(
    const sensor_msgs::msg::PointCloud2& cloud, const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) {
      if (field.count != 1) throw std::invalid_argument("PointCloud2 field must be scalar: " + name);
      return FieldInfo{field.offset, field.datatype, true};
    }
  }
  return {};
}

bool PointCloudOptimizer::read_scalar(const std::uint8_t* data, const FieldInfo& field, double& value) {
  if (!field.present) return false;
  using Field = sensor_msgs::msg::PointField;
  const auto* scalar = data + field.offset;
  switch (field.datatype) {
    case Field::INT8: value = load_scalar<std::int8_t>(scalar); break;
    case Field::UINT8: value = load_scalar<std::uint8_t>(scalar); break;
    case Field::INT16: value = load_scalar<std::int16_t>(scalar); break;
    case Field::UINT16: value = load_scalar<std::uint16_t>(scalar); break;
    case Field::INT32: value = load_scalar<std::int32_t>(scalar); break;
    case Field::UINT32: value = load_scalar<std::uint32_t>(scalar); break;
    case Field::FLOAT32: value = load_scalar<float>(scalar); break;
    case Field::FLOAT64: value = load_scalar<double>(scalar); break;
    default: return false;
  }
  return true;
}

sensor_msgs::msg::PointCloud2 PointCloudOptimizer::optimize(
    const sensor_msgs::msg::PointCloud2& input,
    const std::optional<Eigen::Isometry3d>& cloud_to_height, OptimizerStats* stats) const {
  OptimizerStats counters;
  counters.height_applied = config_.height_filter_enabled && cloud_to_height.has_value();
  if (counters.height_applied && !cloud_to_height->matrix().allFinite()) {
    throw std::invalid_argument("height transform must be finite");
  }
  const FieldInfo x_field = find_field(input, "x");
  const FieldInfo y_field = find_field(input, "y");
  const FieldInfo z_field = find_field(input, "z");
  if (!x_field.present || !y_field.present || !z_field.present) {
    throw std::invalid_argument("PointCloud2 must contain x, y and z fields");
  }
  if (input.point_step == 0 || input.is_bigendian) {
    throw std::invalid_argument("PointCloud2 must be little-endian with non-zero point_step");
  }
  for (const auto& field : {x_field, y_field, z_field}) {
    const std::size_t size = field.datatype == kFloat32 ? sizeof(float) : sizeof(double);
    if ((field.datatype != kFloat32 && field.datatype != kFloat64) ||
        field.offset > input.point_step || size > input.point_step - field.offset) {
      throw std::invalid_argument("invalid PointCloud2 coordinate field layout");
    }
  }
  const FieldInfo intensity_field = config_.min_intensity > 0.0 ? find_field(input, "intensity") : FieldInfo{};
  if (intensity_field.present) {
    const auto size = scalar_size(intensity_field.datatype);
    if (size == 0 || intensity_field.offset > input.point_step || size > input.point_step - intensity_field.offset) {
      throw std::invalid_argument("invalid PointCloud2 intensity field layout");
    }
    counters.intensity_applied = true;
  }

  const std::size_t point_count = static_cast<std::size_t>(input.width) * input.height;
  const std::size_t row_step = input.row_step == 0 ? input.point_step * input.width : input.row_step;
  if (row_step < static_cast<std::size_t>(input.point_step) * input.width ||
      static_cast<std::size_t>(input.height) * row_step > input.data.size()) {
    throw std::invalid_argument("invalid PointCloud2 row layout");
  }
  std::vector<std::size_t> selected;
  selected.reserve(point_count);
  std::unordered_set<VoxelKey, VoxelHash> occupied;
  if (config_.voxel_size > 0.0) occupied.reserve(point_count);

  for (std::size_t index = 0; index < point_count; ++index) {
    const std::size_t row = index / input.width;
    const std::size_t column = index % input.width;
    const std::size_t offset = row * row_step + column * input.point_step;
    if (offset + input.point_step > input.data.size()) continue;
    const auto* point = input.data.data() + offset;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!read_scalar(point, x_field, x) || !read_scalar(point, y_field, y) || !read_scalar(point, z_field, z)) {
      throw std::invalid_argument("x, y and z fields must be FLOAT32 or FLOAT64");
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    const double range_squared = x * x + y * y + z * z;
    if (!std::isfinite(range_squared) || range_squared < config_.min_range * config_.min_range ||
        range_squared > config_.max_range * config_.max_range) {
      continue;
    }

    if (counters.height_applied) {
      const double height = cloud_to_height->linear().row(2).dot(Eigen::Vector3d(x, y, z)) +
                            cloud_to_height->translation().z();
      if (!std::isfinite(height) || height < config_.min_z - kHeightTolerance) {
        ++counters.low_height;
        continue;
      }
      if (height > config_.max_z + kHeightTolerance) {
        ++counters.high_height;
        continue;
      }
    }
    if (counters.intensity_applied) {
      double intensity = 0.0;
      read_scalar(point, intensity_field, intensity);
      if (!std::isfinite(intensity) || intensity < config_.min_intensity) {
        ++counters.low_intensity;
        continue;
      }
    }

    if (config_.voxel_size > 0.0) {
      const VoxelKey key{
          static_cast<std::int64_t>(std::floor(x / config_.voxel_size)),
          static_cast<std::int64_t>(std::floor(y / config_.voxel_size)),
          static_cast<std::int64_t>(std::floor(z / config_.voxel_size))};
      if (!occupied.insert(key).second) continue;
    }
    selected.push_back(offset);
  }

  sensor_msgs::msg::PointCloud2 output = input;
  output.height = 1;
  output.width = static_cast<std::uint32_t>(selected.size());
  output.row_step = output.point_step * output.width;
  output.data.resize(output.row_step);
  for (std::size_t i = 0; i < selected.size(); ++i) {
    std::memcpy(output.data.data() + i * output.point_step, input.data.data() + selected[i], output.point_step);
  }
  output.is_dense = true;
  if (stats) *stats = counters;
  return output;
}

}  // namespace zhao_lio_pointcloud_preprocessor
