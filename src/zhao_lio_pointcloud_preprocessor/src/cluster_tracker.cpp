#include "zhao_lio_pointcloud_preprocessor/cluster_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <nanoflann.hpp>

namespace zhao_lio_pointcloud_preprocessor {
namespace {

using Vector = Eigen::Vector3d;

struct Index {
  const std::vector<Vector>& points;
  using Metric = nanoflann::L2_Simple_Adaptor<double, Index>;
  nanoflann::KDTreeSingleIndexAdaptor<Metric, Index, 3, std::size_t> tree;
  explicit Index(const std::vector<Vector>& source) : points(source), tree(3, *this, {10}) {
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

struct Component {
  std::vector<std::size_t> ids;
  Vector minimum = Vector::Constant(std::numeric_limits<double>::infinity());
  Vector maximum = Vector::Constant(-std::numeric_limits<double>::infinity());
};

double median(std::vector<double>& values) {
  auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

struct Fit {
  Vector displacement = Vector::Zero();
  double static_rmse = 0.0;
  double motion_rmse = 0.0;
  double overlap = 0.0;
};

Fit fit_translation(const std::vector<Vector>& reference, const std::vector<Vector>& current,
                    const Vector& initial, double radius) {
  Index reference_index(reference);
  Fit result;
  result.displacement = initial;
  for (int iteration = 0; iteration < 8; ++iteration) {
    std::vector<double> residuals[3];
    for (const auto& point : current) {
      const Vector query = point - result.displacement;
      std::size_t nearest = 0;
      if (reference_index.nearest(query, nearest) < radius) {
        const Vector residual = point - reference[nearest];
        for (int axis = 0; axis < 3; ++axis) residuals[axis].push_back(residual[axis]);
      }
    }
    if (residuals[0].size() < 3) break;
    Vector updated;
    for (int axis = 0; axis < 3; ++axis) updated[axis] = median(residuals[axis]);
    const double change = (updated - result.displacement).norm();
    result.displacement = updated;
    if (change < 0.002) break;
  }
  Index current_index(current);
  std::size_t overlap = 0;
  double static_error = 0.0;
  double motion_error = 0.0;
  const auto accumulate = [&](const std::vector<Vector>& source, const Index& target, const Vector& shift) {
    for (const auto& point : source) {
      std::size_t nearest = 0;
      const double baseline = target.nearest(point, nearest);
      const double compensated = target.nearest(point + shift, nearest);
      static_error += std::min(baseline * baseline, 4.0 * radius * radius);
      motion_error += std::min(compensated * compensated, 4.0 * radius * radius);
      if (compensated < radius) ++overlap;
    }
  };
  accumulate(current, reference_index, -result.displacement);
  accumulate(reference, current_index, result.displacement);
  const double count = reference.size() + current.size();
  result.static_rmse = std::sqrt(static_error / count);
  result.motion_rmse = std::sqrt(motion_error / count);
  result.overlap = overlap / count;
  return result;
}

}

ClusterTracker::ClusterTracker(ClusterConfig config) : config_(config) {
  const double positive[] = {config.tolerance, config.ground_clearance, config.max_extent,
      config.association_distance, config.association_ambiguity, config.motion_window,
      config.min_displacement, config.min_speed, config.max_speed, config.fit_distance,
      config.max_fit_rmse, config.min_fit_improvement, config.min_overlap, config.track_timeout};
  for (double value : positive) {
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument("invalid cluster tracking parameter");
  }
  if (!std::isfinite(config.ground_z) || config.min_points < 3 || config.confirmation_frames < 2 ||
      config.min_overlap > 1.0 || config.min_speed >= config.max_speed || config.motion_window > 1.0 ||
      config.track_timeout > 1.0 || config.ground_clearance <= 0.02) {
    throw std::invalid_argument("inconsistent cluster tracking parameters");
  }
}

void ClusterTracker::reset() {
  tracks_.clear();
  world_from_lidar_.setIdentity();
  last_time_ = -1.0;
}

ClusterResult ClusterTracker::update(const std::vector<Vector>& points, double time,
                                   const Eigen::Isometry3d& current_to_previous,
                                   const Eigen::Isometry3d& lidar_to_base) {
  if (!std::isfinite(time) || !current_to_previous.matrix().allFinite() || !lidar_to_base.matrix().allFinite()) {
    reset();
    throw std::invalid_argument("invalid cluster time or transform");
  }
  if (last_time_ >= 0.0 && (time <= last_time_ || time - last_time_ > config_.track_timeout)) reset();
  if (last_time_ >= 0.0) world_from_lidar_ = world_from_lidar_ * current_to_previous;
  last_time_ = time;
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& track) {
    return time - track.history.back().time > config_.track_timeout;
  }), tracks_.end());
  ClusterResult result;
  result.remove.resize(points.size(), false);
  std::vector<Vector> foreground;
  std::vector<std::size_t> foreground_ids;
  std::vector<double> heights(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (!points[index].allFinite()) {
      reset();
      throw std::invalid_argument("invalid cluster point");
    }
    heights[index] = (lidar_to_base * points[index]).z();
    if (heights[index] > config_.ground_z + config_.ground_clearance) {
      foreground.push_back(points[index]);
      foreground_ids.push_back(index);
    }
  }
  Index spatial(foreground);
  std::vector<bool> visited(foreground.size(), false);
  std::vector<Component> components;
  std::vector<int> labels(foreground.size(), -1);
  for (std::size_t seed = 0; seed < foreground.size(); ++seed) {
    if (visited[seed]) continue;
    std::vector<std::size_t> queue{seed};
    visited[seed] = true;
    Component component;
    for (std::size_t cursor = 0; cursor < queue.size(); ++cursor) {
      const auto index = queue[cursor];
      component.ids.push_back(foreground_ids[index]);
      component.minimum = component.minimum.cwiseMin(foreground[index]);
      component.maximum = component.maximum.cwiseMax(foreground[index]);
      std::vector<std::pair<std::size_t, double>> neighbors;
      spatial.tree.radiusSearch(foreground[index].data(), config_.tolerance * config_.tolerance,
                                neighbors, nanoflann::SearchParams());
      for (const auto& neighbor : neighbors) {
        if (!visited[neighbor.first]) {
          visited[neighbor.first] = true;
          queue.push_back(neighbor.first);
        }
      }
    }
    if (component.ids.size() < static_cast<std::size_t>(config_.min_points) ||
        (component.maximum - component.minimum).maxCoeff() > config_.max_extent) {
      ++result.rejected_components;
      continue;
    }
    for (const auto index : queue) labels[index] = static_cast<int>(components.size());
    components.push_back(std::move(component));
  }
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (heights[index] <= config_.ground_z + 0.02 || heights[index] > config_.ground_z + config_.ground_clearance) continue;
    std::size_t nearest = 0;
    if (spatial.nearest(points[index], nearest) < config_.tolerance && labels[nearest] >= 0) {
      auto& component = components[labels[nearest]];
      component.ids.push_back(index);
      component.minimum = component.minimum.cwiseMin(points[index]);
      component.maximum = component.maximum.cwiseMax(points[index]);
    }
  }
  std::vector<Observation> observations;
  for (const auto& component : components) {
    Observation observation;
    observation.time = time;
    Vector minimum = Vector::Constant(std::numeric_limits<double>::infinity());
    Vector maximum = -minimum;
    for (const auto index : component.ids) {
      const Vector point = world_from_lidar_ * points[index];
      observation.center += point;
      minimum = minimum.cwiseMin(point);
      maximum = maximum.cwiseMax(point);
    }
    observation.center /= component.ids.size();
    observation.extent = maximum - minimum;
    const std::size_t stride = std::max<std::size_t>(1, (component.ids.size() + 255) / 256);
    for (std::size_t index = 0; index < component.ids.size(); index += stride) {
      observation.points.push_back(world_from_lidar_ * points[component.ids[index]]);
    }
    observations.push_back(std::move(observation));
  }
  const std::size_t old_track_count = tracks_.size();
  std::vector<int> preferred(observations.size(), -1);
  std::vector<double> best_score(observations.size(), std::numeric_limits<double>::infinity());
  std::vector<double> best_for_track(old_track_count, std::numeric_limits<double>::infinity());
  std::vector<int> preferred_for_track(old_track_count, -1);
  for (std::size_t component = 0; component < observations.size(); ++component) {
    double second = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < old_track_count; ++index) {
      const auto& track = tracks_[index];
      const auto& last = track.history.back();
      const double elapsed = time - last.time;
      const Vector predicted = last.center + track.velocity * elapsed;
      const double distance = (observations[component].center - predicted).norm();
      const double shape_change = (observations[component].extent - last.extent).norm();
      if (distance > config_.association_distance + config_.max_speed * elapsed || shape_change > 0.8) continue;
      const double score = distance + 0.3 * shape_change;
      if (score < best_score[component]) {
        second = best_score[component];
        best_score[component] = score;
        preferred[component] = index;
      } else {
        second = std::min(second, score);
      }
      if (score < best_for_track[index]) {
        best_for_track[index] = score;
        preferred_for_track[index] = component;
      }
    }
    if (second - best_score[component] < config_.association_ambiguity) preferred[component] = -1;
  }
  for (std::size_t component = 0; component < observations.size(); ++component) {
    auto& observation = observations[component];
    int index = preferred[component];
    if (index < 0 || preferred_for_track[index] != static_cast<int>(component)) {
      if (tracks_.size() >= 128) continue;
      Track track;
      track.id = next_id_++;
      tracks_.push_back(std::move(track));
      index = tracks_.size() - 1;
    }
    auto& track = tracks_[index];
    ClusterView view;
    view.id = track.id;
    view.minimum = components[component].minimum;
    view.maximum = components[component].maximum;
    view.points = components[component].ids.size();
    const Observation* reference = nullptr;
    for (auto sample = track.history.rbegin(); sample != track.history.rend(); ++sample) {
      if (time - sample->time >= config_.motion_window - 1e-6) {
        reference = &*sample;
        break;
      }
    }
    if (reference && time - reference->time <= config_.motion_window + config_.track_timeout) {
      const Fit fit = fit_translation(reference->points, observation.points,
                                      observation.center - reference->center, config_.fit_distance);
      const double elapsed = time - reference->time;
      const Vector velocity = fit.displacement / elapsed;
      view.speed = velocity.norm();
      view.displacement = fit.displacement.norm();
      view.static_rmse = fit.static_rmse;
      view.motion_rmse = fit.motion_rmse;
      const bool shape_valid = (observation.extent - reference->extent).norm() <= 0.60 &&
          fit.overlap >= config_.min_overlap && fit.motion_rmse <= config_.max_fit_rmse;
      const bool moving = shape_valid && view.displacement >= config_.min_displacement &&
          view.speed >= config_.min_speed && view.speed <= config_.max_speed &&
          fit.static_rmse - fit.motion_rmse >= config_.min_fit_improvement;
      const bool consistent = track.velocity.norm() < config_.min_speed ||
          velocity.dot(track.velocity) > 0.5 * velocity.norm() * track.velocity.norm();
      const bool uninterrupted = time - track.history.back().time <= 0.20;
      if (moving) {
        track.confirmations = consistent && uninterrupted ? track.confirmations + 1 : 1;
        track.confirmations = std::min(track.confirmations, config_.confirmation_frames);
        track.velocity = velocity;
        view.state = "motion_pending";
      } else {
        track.confirmations = 0;
        track.velocity.setZero();
        view.state = shape_valid ? "stationary" : "shape_uncertain";
      }
      view.confirmations = track.confirmations;
      view.moving = track.confirmations >= config_.confirmation_frames;
      if (view.moving) {
        view.state = "moving";
        for (const auto point : components[component].ids) result.remove[point] = true;
      }
    }
    track.history.push_back(std::move(observation));
    while (track.history.size() > 32 ||
           (track.history.size() > 1 && time - track.history.front().time > config_.motion_window + config_.track_timeout)) {
      track.history.pop_front();
    }
    result.clusters.push_back(std::move(view));
  }
  return result;
}

}
