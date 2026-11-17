#include "zhao_lio_pointcloud_preprocessor/dynamic_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <nanoflann.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace zhao_lio_pointcloud_preprocessor {
namespace {

using Cloud = sensor_msgs::msg::PointCloud2;
using Vector = Eigen::Vector3d;
constexpr double pi = 3.14159265358979323846;

double stamp(const Cloud& cloud) {
  return cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9;
}

struct Layout {
  std::uint32_t coordinates[3];
  std::uint32_t time;
  std::size_t count;
  std::size_t row_step;
  explicit Layout(const Cloud& cloud) {
    if (cloud.is_bigendian || cloud.point_step == 0) {
      throw std::invalid_argument("dynamic filter requires little-endian PointCloud2");
    }
    const auto field_offset = [&cloud](const std::string& name, int type, unsigned size) {
      for (const auto& field : cloud.fields) {
        if (field.name == name && field.datatype == type && field.count == 1 &&
            field.offset <= cloud.point_step && size <= cloud.point_step - field.offset) {
          return field.offset;
        }
      }
      throw std::invalid_argument("invalid or missing field: " + name);
    };
    coordinates[0] = field_offset("x", sensor_msgs::msg::PointField::FLOAT32, 4);
    coordinates[1] = field_offset("y", sensor_msgs::msg::PointField::FLOAT32, 4);
    coordinates[2] = field_offset("z", sensor_msgs::msg::PointField::FLOAT32, 4);
    time = field_offset("timestamp", sensor_msgs::msg::PointField::FLOAT64, 8);
    count = static_cast<std::size_t>(cloud.width) * cloud.height;
    row_step = cloud.row_step;
    if (row_step < static_cast<std::size_t>(cloud.width) * cloud.point_step ||
        static_cast<std::size_t>(cloud.height) * row_step > cloud.data.size()) {
      throw std::invalid_argument("truncated PointCloud2 layout");
    }
  }
  const std::uint8_t* data(const Cloud& cloud, std::size_t index) const {
    return cloud.data.data() + (index / cloud.width) * row_step +
           (index % cloud.width) * cloud.point_step;
  }
  double point_time(const Cloud& cloud, std::size_t index) const {
    double value;
    std::memcpy(&value, data(cloud, index) + time, sizeof(value));
    return value > 1e12 ? value * 1e-9 : value;
  }
  Vector point(const Cloud& cloud, std::size_t index) const {
    Vector value;
    for (int axis = 0; axis < 3; ++axis) {
      float component;
      std::memcpy(&component, data(cloud, index) + coordinates[axis], sizeof(component));
      value[axis] = component;
    }
    return value;
  }
};

Cloud empty_cloud(const Cloud& source) {
  Cloud output = source;
  output.width = 0;
  output.height = 1;
  output.row_step = 0;
  output.data.clear();
  return output;
}

struct PointIndex {
  const std::vector<Vector>& points;
  using Metric = nanoflann::L2_Simple_Adaptor<double, PointIndex>;
  using Tree = nanoflann::KDTreeSingleIndexAdaptor<Metric, PointIndex, 3, std::size_t>;
  Tree tree;
  explicit PointIndex(const std::vector<Vector>& values)
      : points(values), tree(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10)) {
    tree.buildIndex();
  }
  std::size_t kdtree_get_point_count() const { return points.size(); }
  double kdtree_get_pt(std::size_t index, std::size_t axis) const { return points[index][axis]; }
  template <class Bounds> bool kdtree_get_bbox(Bounds&) const { return false; }
  double nearest(const Vector& query, std::size_t& index) const {
    double distance = std::numeric_limits<double>::infinity();
    if (!points.empty()) tree.knnSearch(query.data(), 1, &index, &distance);
    return std::sqrt(distance);
  }
};

Eigen::Quaterniond increment(const Vector& angle) {
  const double norm = angle.norm();
  return norm < 1e-12 ? Eigen::Quaterniond::Identity() :
      Eigen::Quaterniond(Eigen::AngleAxisd(norm, angle / norm));
}

double median(std::vector<double>& values) {
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

class Visibility {
 public:
  Visibility(const std::vector<Vector>& points, double degrees)
      : bins_(static_cast<int>(std::ceil(360.0 / degrees))), step_(2.0 * pi / bins_) {
    for (const auto& point : points) {
      const auto bin = coordinates(point);
      auto inserted = ranges_.emplace(key(bin.first, bin.second), point.norm());
      if (!inserted.second) inserted.first->second = std::min(inserted.first->second, point.norm());
    }
  }
  bool was_free(const Vector& point, double margin, int min_bins) const {
    const auto bin = coordinates(point);
    int covered = 0;
    double nearest_range = std::numeric_limits<double>::infinity();
    for (int azimuth = -1; azimuth <= 1; ++azimuth) {
      for (int elevation = -1; elevation <= 1; ++elevation) {
        auto found = ranges_.find(key(bin.first + azimuth, bin.second + elevation));
        if (found != ranges_.end()) {
          ++covered;
          nearest_range = std::min(nearest_range, found->second);
        }
      }
    }
    return covered >= min_bins && point.norm() + margin < nearest_range;
  }
 private:
  std::pair<int, int> coordinates(const Vector& point) const {
    return {static_cast<int>(std::floor((std::atan2(point.y(), point.x()) + pi) / step_)),
            static_cast<int>(std::floor((std::atan2(point.z(), std::hypot(point.x(), point.y())) + pi / 2) / step_))};
  }
  std::int64_t key(int azimuth, int elevation) const {
    return static_cast<std::int64_t>(elevation + bins_) * bins_ + (azimuth % bins_ + bins_) % bins_;
  }
  int bins_;
  double step_;
  std::unordered_map<std::int64_t, double> ranges_;
};

}

DynamicFilter::DynamicFilter(DynamicConfig config) : config_(config), cluster_tracker_(config.cluster) {
  const double values[] = {config.min_dt, config.max_dt, config.max_scan_duration, config.max_imu_gap,
      config.correspondence_distance, config.static_distance, config.registration_inlier_distance, config.max_registration_rmse,
      config.min_inlier_ratio, config.max_ego_speed, config.angular_bin_deg, config.free_space_margin,
      config.cluster_radius, config.max_object_speed, config.max_removed_ratio};
  for (double value : values) {
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument("invalid dynamic filter parameter");
  }
  if (config.max_dt <= config.min_dt || config.max_dt > 1.0 || config.max_scan_duration > 1.0 ||
      config.max_imu_gap > config.max_dt || config.min_inlier_ratio > 1.0 ||
      config.max_removed_ratio > 0.5 || config.static_distance >= config.correspondence_distance ||
      config.registration_inlier_distance < config.static_distance ||
      config.registration_inlier_distance > config.correspondence_distance ||
      config.angular_bin_deg < 0.1 || config.angular_bin_deg > 10.0 || config.min_points < 3 ||
      config.min_cluster_points < 2 || config.confirmation_frames < 2 ||
      config.min_visibility_bins < 1 || config.min_visibility_bins > 9) {
    throw std::invalid_argument("inconsistent dynamic filter parameters");
  }
}

void DynamicFilter::reset() {
  cluster_tracker_.reset();
  imu_.clear();
  previous_ = Scan{};
  velocity_.setZero();
  frame_.clear();
}

bool DynamicFilter::add_imu(double time, const Vector& gyro_lidar) {
  if (!std::isfinite(time) || !gyro_lidar.allFinite()) {
    reset();
    return false;
  }
  if (!imu_.empty() && time == imu_.back().time) return false;
  if (!imu_.empty() && (time < imu_.back().time || time - imu_.back().time > config_.max_imu_gap)) reset();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  if (!imu_.empty()) {
    const auto& last = imu_.back();
    orientation = (last.orientation * increment(0.5 * (last.gyro + gyro_lidar) * (time - last.time))).normalized();
  }
  imu_.push_back({time, gyro_lidar, orientation});
  while (imu_.size() > 2 && (time - imu_.front().time > 2.0 || imu_.size() > 4000)) imu_.pop_front();
  return true;
}

bool DynamicFilter::has_imu_until(double time) const {
  return !imu_.empty() && imu_.back().time >= time;
}

bool DynamicFilter::orientation_at(double time, Eigen::Quaterniond& orientation) const {
  if (imu_.empty() || time < imu_.front().time || time > imu_.back().time) return false;
  auto upper = std::lower_bound(imu_.begin(), imu_.end(), time,
      [](const ImuPose& sample, double value) { return sample.time < value; });
  if (upper == imu_.begin() || upper->time == time) {
    orientation = upper->orientation;
  } else {
    const auto lower = std::prev(upper);
    orientation = lower->orientation.slerp((time - lower->time) / (upper->time - lower->time), upper->orientation);
  }
  return true;
}

double DynamicFilter::scan_end(const Cloud& cloud) const {
  const Layout layout(cloud);
  const double begin = stamp(cloud);
  double end = begin;
  for (std::size_t index = 0; index < layout.count; ++index) {
    const double time = layout.point_time(cloud, index);
    if (!std::isfinite(time) || time < begin - 1e-6 || time > begin + config_.max_scan_duration) {
      throw std::invalid_argument("point timestamp must be absolute seconds/ns within scan duration");
    }
    end = std::max(end, time);
  }
  return end;
}

DynamicResult DynamicFilter::bypass(const Cloud& cloud, const std::string& reason) {
  cluster_tracker_.reset();
  previous_ = Scan{};
  velocity_.setZero();
  DynamicResult result;
  result.kept = cloud;
  result.removed = empty_cloud(cloud);
  result.reason = reason;
  return result;
}

DynamicResult DynamicFilter::filter(const Cloud& cloud, const std::optional<Eigen::Isometry3d>& lidar_to_base) {
  if (!config_.enabled) return bypass(cloud, "disabled");
  if (config_.cluster_tracking_enabled && !lidar_to_base) return bypass(cloud, "cluster_tf_unavailable");
  const Layout layout(cloud);
  const double end = scan_end(cloud);
  if (!frame_.empty() && frame_ != cloud.header.frame_id) {
    reset();
    return bypass(cloud, "frame_changed");
  }
  frame_ = cloud.header.frame_id;
  if (!previous_.points.empty() && end <= previous_.time) {
    reset();
    return bypass(cloud, "time_reversed");
  }
  if (layout.count < static_cast<std::size_t>(config_.min_points)) return bypass(cloud, "sparse_scan");
  Scan current;
  current.time = end;
  if (!orientation_at(stamp(cloud), current.orientation) || !orientation_at(end, current.orientation)) {
    return bypass(cloud, "imu_not_bracketed");
  }
  current.points.reserve(layout.count);
  current.confirmations.resize(layout.count, 0);
  const Eigen::Quaterniond relative = previous_.orientation.conjugate() * current.orientation;
  const Vector scan_velocity = relative.conjugate() * velocity_;
  for (std::size_t index = 0; index < layout.count; ++index) {
    Eigen::Quaterniond point_orientation;
    const double time = layout.point_time(cloud, index);
    const Vector point = layout.point(cloud, index);
    if (!point.allFinite() || !orientation_at(time, point_orientation)) return bypass(cloud, "invalid_point_or_imu");
    current.points.push_back(current.orientation.conjugate() * point_orientation * point + scan_velocity * (time - end));
  }
  DynamicResult result;
  result.kept = cloud;
  result.removed = empty_cloud(cloud);
  result.reason = "warmup";
  result.reference_time = end;
  if (previous_.points.empty()) {
    if (config_.cluster_tracking_enabled) {
      auto clusters = cluster_tracker_.update(current.points, end, Eigen::Isometry3d::Identity(), *lidar_to_base);
      result.clusters = std::move(clusters.clusters);
      result.rejected_components = clusters.rejected_components;
    }
    previous_ = std::move(current);
    return result;
  }
  result.dt = end - previous_.time;
  if (result.dt < config_.min_dt) {
    result.reason = "dt_too_short";
    return result;
  }
  if (result.dt > config_.max_dt) {
    cluster_tracker_.reset();
    previous_ = std::move(current);
    velocity_.setZero();
    result.reason = "dt_too_long";
    return result;
  }
  PointIndex previous_index(previous_.points);
  std::vector<Vector> rotated;
  rotated.reserve(layout.count);
  for (const auto& point : current.points) rotated.push_back(relative * point);
  Vector translation = velocity_ * result.dt;
  const std::size_t stride = std::max<std::size_t>(1, layout.count / 3000);
  bool converged = false;
  for (int iteration = 0; iteration < 12; ++iteration) {
    std::vector<double> residuals[3];
    for (std::size_t index = 0; index < rotated.size(); index += stride) {
      const Vector query = rotated[index] + translation;
      std::size_t nearest = 0;
      if (previous_index.nearest(query, nearest) < config_.correspondence_distance) {
        const Vector residual = previous_.points[nearest] - query;
        for (int axis = 0; axis < 3; ++axis) residuals[axis].push_back(residual[axis]);
      }
    }
    if (residuals[0].size() < static_cast<std::size_t>(config_.min_points / 2)) break;
    Vector delta;
    for (int axis = 0; axis < 3; ++axis) delta[axis] = median(residuals[axis]);
    translation += delta;
    if (delta.norm() < 0.005) {
      converged = true;
      break;
    }
  }
  std::vector<double> distances(layout.count);
  std::vector<Vector> aligned;
  aligned.reserve(layout.count);
  std::size_t inliers = 0;
  double squared_error = 0.0;
  for (std::size_t index = 0; index < layout.count; ++index) {
    aligned.push_back(rotated[index] + translation);
    std::size_t nearest = 0;
    distances[index] = previous_index.nearest(aligned.back(), nearest);
    if (distances[index] < config_.registration_inlier_distance) {
      ++inliers;
      squared_error += distances[index] * distances[index];
    }
  }
  result.inlier_ratio = static_cast<double>(inliers) / layout.count;
  result.registration_converged = converged;
  result.registration_rmse = inliers == 0 ? std::numeric_limits<double>::infinity() : std::sqrt(squared_error / inliers);
  if (!converged || result.inlier_ratio < config_.min_inlier_ratio ||
      result.registration_rmse > config_.max_registration_rmse || translation.norm() > config_.max_ego_speed * result.dt) {
    cluster_tracker_.reset();
    previous_ = std::move(current);
    velocity_.setZero();
    result.reason = "registration_unreliable";
    return result;
  }
  velocity_ = relative.conjugate() * (translation / result.dt);
  std::vector<bool> remove(layout.count, false);
  std::size_t removed_count = 0;
  if (config_.cluster_tracking_enabled) {
    Eigen::Isometry3d current_to_previous = Eigen::Isometry3d::Identity();
    current_to_previous.linear() = relative.toRotationMatrix();
    current_to_previous.translation() = translation;
    auto clusters = cluster_tracker_.update(current.points, end, current_to_previous, *lidar_to_base);
    remove = std::move(clusters.remove);
    result.clusters = std::move(clusters.clusters);
    result.rejected_components = clusters.rejected_components;
    for (const auto& cluster : result.clusters) {
      result.clustered += cluster.points;
      if (cluster.confirmations > 0) result.candidates += cluster.points;
    }
    removed_count = std::count(remove.begin(), remove.end(), true);
  } else {
    Visibility visibility(previous_.points, config_.angular_bin_deg);
    std::vector<Vector> old_candidates;
    std::vector<int> old_confirmations;
    for (std::size_t index = 0; index < previous_.points.size(); ++index) {
      if (previous_.confirmations[index] > 0) {
        old_candidates.push_back(previous_.points[index]);
        old_confirmations.push_back(previous_.confirmations[index]);
      }
    }
    PointIndex candidate_index(old_candidates);
    std::vector<Vector> candidates;
    std::vector<std::size_t> candidate_ids;
    for (std::size_t index = 0; index < layout.count; ++index) {
      if (distances[index] <= config_.static_distance) continue;
      ++result.unmatched;
      if (!visibility.was_free(aligned[index], config_.free_space_margin, config_.min_visibility_bins)) continue;
      current.confirmations[index] = 1;
      std::size_t nearest = 0;
      if (candidate_index.nearest(aligned[index], nearest) < config_.max_object_speed * result.dt) {
        current.confirmations[index] = std::min(config_.confirmation_frames, old_confirmations[nearest] + 1);
      }
      candidates.push_back(aligned[index]);
      candidate_ids.push_back(index);
    }
    result.candidates = candidates.size();
    PointIndex cluster_index(candidates);
    for (std::size_t candidate = 0; candidate < candidates.size(); ++candidate) {
      const auto index = candidate_ids[candidate];
      std::vector<std::pair<std::size_t, double>> neighbors;
      cluster_index.tree.radiusSearch(candidates[candidate].data(), config_.cluster_radius * config_.cluster_radius,
                                     neighbors, nanoflann::SearchParams());
      if (neighbors.size() < static_cast<std::size_t>(config_.min_cluster_points)) {
        current.confirmations[index] = 0;
      } else {
        ++result.clustered;
        if (current.confirmations[index] >= config_.confirmation_frames) {
          remove[index] = true;
          ++removed_count;
        }
      }
    }
  }
  result.reason = "matched";
  result.confirmed = removed_count;
  if (static_cast<double>(removed_count) / layout.count > config_.max_removed_ratio) {
    std::fill(current.confirmations.begin(), current.confirmations.end(), 0);
    result.reason = "removal_limit";
  } else {
    for (auto& cluster : result.clusters) cluster.removed = cluster.moving;
    result.kept = empty_cloud(cloud);
    result.kept.data.reserve(cloud.data.size());
    result.removed.data.reserve(removed_count * cloud.point_step);
    for (std::size_t index = 0; index < layout.count; ++index) {
      auto& output = remove[index] ? result.removed : result.kept;
      const auto* point = layout.data(cloud, index);
      output.data.insert(output.data.end(), point, point + cloud.point_step);
      ++output.width;
    }
    result.kept.row_step = result.kept.width * cloud.point_step;
    result.removed.row_step = result.removed.width * cloud.point_step;
  }
  previous_ = std::move(current);
  return result;
}

}
