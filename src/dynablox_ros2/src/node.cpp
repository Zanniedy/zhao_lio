#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "dynablox_ros2/adapter.hpp"

namespace dynablox_ros2 {
namespace {
using Cloud = sensor_msgs::msg::PointCloud2;
using Vector = Eigen::Vector3d;
using Clock = std::chrono::steady_clock;

struct Layout {
  std::uint32_t coordinates[3], timestamp;
  std::size_t count;
  double begin, end;
  explicit Layout(const Cloud& cloud) {
    if (cloud.is_bigendian || cloud.point_step == 0 ||
        cloud.row_step < static_cast<std::size_t>(cloud.width) * cloud.point_step ||
        static_cast<std::size_t>(cloud.row_step) * cloud.height > cloud.data.size()) {
      throw std::invalid_argument("invalid little-endian cloud layout");
    }
    const auto offset = [&](const std::string& name, int datatype, unsigned bytes) {
      for (const auto& field : cloud.fields) {
        if (field.name == name && field.datatype == datatype && field.count == 1 &&
            field.offset <= cloud.point_step && bytes <= cloud.point_step - field.offset) return field.offset;
      }
      throw std::invalid_argument("missing or invalid field: " + name);
    };
    coordinates[0] = offset("x", sensor_msgs::msg::PointField::FLOAT32, 4);
    coordinates[1] = offset("y", sensor_msgs::msg::PointField::FLOAT32, 4);
    coordinates[2] = offset("z", sensor_msgs::msg::PointField::FLOAT32, 4);
    timestamp = offset("timestamp", sensor_msgs::msg::PointField::FLOAT64, 8);
    count = static_cast<std::size_t>(cloud.width) * cloud.height;
    begin = end = rclcpp::Time(cloud.header.stamp).seconds();
    for (std::size_t index = 0; index < count; ++index) {
      const double stamp = time(cloud, index);
      if (!std::isfinite(stamp) || stamp < begin - 1e-6 || stamp > begin + 0.15) {
        throw std::invalid_argument("point timestamp outside scan");
      }
      end = std::max(end, stamp);
    }
  }
  const std::uint8_t* data(const Cloud& cloud, std::size_t index) const {
    return cloud.data.data() + index / cloud.width * cloud.row_step + index % cloud.width * cloud.point_step;
  }
  double time(const Cloud& cloud, std::size_t index) const {
    double value;
    std::memcpy(&value, data(cloud, index) + timestamp, sizeof(value));
    return value > 1e12 ? value * 1e-9 : value;
  }
  Vector point(const Cloud& cloud, std::size_t index) const {
    Vector result;
    for (int axis = 0; axis < 3; ++axis) {
      float value;
      std::memcpy(&value, data(cloud, index) + coordinates[axis], sizeof(value));
      result[axis] = value;
    }
    return result;
  }
};

Cloud subset(const Cloud& source, const Layout& layout, const std::vector<PointLabel>& labels,
             PointLabel label) {
  Cloud output;
  output.header = source.header;
  output.fields = source.fields;
  output.point_step = source.point_step;
  output.is_bigendian = source.is_bigendian;
  output.is_dense = source.is_dense;
  output.height = 1;
  for (std::size_t index = 0; index < labels.size(); ++index) {
    if (labels[index] != label) continue;
    const auto record = layout.data(source, index);
    output.data.insert(output.data.end(), record, record + source.point_step);
    ++output.width;
  }
  output.row_step = output.width * output.point_step;
  return output;
}

Cloud world_cloud(const std::vector<Vector>& points, const std_msgs::msg::Header& header) {
  Cloud cloud;
  cloud.header = header;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> horizontal(cloud, "x"), lateral(cloud, "y"), vertical(cloud, "z");
  for (const auto& point : points) {
    *horizontal = point.x(); *lateral = point.y(); *vertical = point.z();
    ++horizontal; ++lateral; ++vertical;
  }
  return cloud;
}

Eigen::Isometry3d pose(const geometry_msgs::msg::TransformStamped& transform) {
  const auto& rotation = transform.transform.rotation;
  Eigen::Quaterniond quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
  if (!quaternion.coeffs().allFinite() || std::abs(quaternion.norm() - 1.0) > 1e-3) {
    throw std::invalid_argument("invalid TF rotation");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = quaternion.normalized().toRotationMatrix();
  const auto& translation = transform.transform.translation;
  result.translation() = Vector(translation.x, translation.y, translation.z);
  if (!result.matrix().allFinite()) throw std::invalid_argument("invalid TF translation");
  return result;
}
}

class DynabloxNode : public rclcpp::Node {
 public:
  DynabloxNode() : Node("dynablox") {
    DynabloxConfig config;
    config.resolution = declare_parameter("resolution", config.resolution);
    config.min_range = declare_parameter("min_range", config.min_range);
    config.max_range = declare_parameter("max_range", config.max_range);
    config.map_radius = declare_parameter("map_radius", config.map_radius);
    config.max_time_gap = declare_parameter("max_time_gap", config.max_time_gap);
    config.voxels_per_side = declare_parameter("voxels_per_side", config.voxels_per_side);
    config.max_blocks = declare_parameter("max_blocks", config.max_blocks);
    config.num_threads = declare_parameter("num_threads", config.num_threads);
    config.integration_method = declare_parameter("integration_method", config.integration_method);
    config.ever_free.counter_to_reset = declare_parameter("ever_free.counter_to_reset", config.ever_free.counter_to_reset);
    config.ever_free.temporal_buffer = declare_parameter("ever_free.temporal_buffer", config.ever_free.temporal_buffer);
    config.ever_free.burn_in_period = declare_parameter("ever_free.burn_in_period", config.ever_free.burn_in_period);
    config.ever_free.neighbor_connectivity = declare_parameter("ever_free.neighbor_connectivity", config.ever_free.neighbor_connectivity);
    config.clustering.min_cluster_size = declare_parameter("clustering.min_cluster_size", config.clustering.min_cluster_size);
    config.clustering.max_cluster_size = declare_parameter("clustering.max_cluster_size", config.clustering.max_cluster_size);
    config.clustering.neighbor_connectivity = declare_parameter("clustering.neighbor_connectivity", config.clustering.neighbor_connectivity);
    config.clustering.grow_clusters_twice = declare_parameter("clustering.grow_clusters_twice", config.clustering.grow_clusters_twice);
    config.clustering.check_cluster_separation_exact = declare_parameter("clustering.check_cluster_separation_exact", config.clustering.check_cluster_separation_exact);
    config.tracking.min_track_duration = declare_parameter("tracking.min_track_duration", config.tracking.min_track_duration);
    config.ever_free.tsdf_occupancy_threshold = declare_parameter<double>("ever_free.tsdf_occupancy_threshold", config.ever_free.tsdf_occupancy_threshold);
    config.clustering.min_extent = declare_parameter<double>("clustering.min_extent", config.clustering.min_extent);
    config.clustering.max_extent = declare_parameter<double>("clustering.max_extent", config.clustering.max_extent);
    config.clustering.min_cluster_separation = declare_parameter<double>("clustering.min_cluster_separation", config.clustering.min_cluster_separation);
    config.tracking.max_tracking_distance = declare_parameter<double>("tracking.max_tracking_distance", config.tracking.max_tracking_distance);
    filter_ = std::make_unique<DynabloxAdapter>(config);
    world_frame_ = declare_parameter<std::string>("world_frame", "lio_odom");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "mid70_link");
    const auto input = declare_parameter<std::string>("input_topic", "/input/lidar");
    tf_wait_ = declare_parameter<double>("tf_wait", 0.6);
    max_speed_ = declare_parameter<double>("max_pose_speed", 3.0);
    max_angular_speed_ = declare_parameter<double>("max_pose_angular_speed", 3.0);
    if (tf_wait_ <= 0.0 || max_speed_ <= 0.0 || max_angular_speed_ <= 0.0 ||
        !std::isfinite(tf_wait_) || !std::isfinite(max_speed_) || !std::isfinite(max_angular_speed_) ||
        world_frame_.empty() || lidar_frame_.empty()) throw std::invalid_argument("invalid node parameters");
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    auto live = rclcpp::SensorDataQoS();
    live.keep_last(2);
    kept_ = create_publisher<Cloud>("/dynablox/kept", live);
    removed_ = create_publisher<Cloud>("/dynablox/dynamic", live);
    pending_cloud_ = create_publisher<Cloud>("/dynablox/pending", live);
    scan_ = create_publisher<Cloud>("/dynablox/scan_world", live);
    static_scan_ = create_publisher<Cloud>("/dynablox/static_world", live);
    dynamic_scan_ = create_publisher<Cloud>("/dynablox/dynamic_world", live);
    pending_scan_ = create_publisher<Cloud>("/dynablox/pending_world", live);
    seeds_ = create_publisher<Cloud>("/dynablox/seeds", live);
    map_ = create_publisher<Cloud>("/dynablox/static_map", rclcpp::QoS(1).transient_local());
    free_ = create_publisher<Cloud>("/dynablox/free", rclcpp::QoS(1).transient_local());
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/dynablox/clusters", rclcpp::QoS(1).transient_local());
    status_ = create_publisher<std_msgs::msg::String>("/dynablox/status", 10);
    subscription_ = create_subscription<Cloud>(input, rclcpp::SensorDataQoS().keep_last(5),
        [this](Cloud::ConstSharedPtr message) {
          const double time = rclcpp::Time(message->header.stamp).seconds();
          if (time <= last_received_) {
            queue_.clear();
            filter_->reset();
            last_pose_time_ = -1.0;
            last_debug_ = -1.0;
          }
          last_received_ = time;
          if (queue_.size() >= 8) {
            bypass(*queue_.front().cloud, "queue_limit");
            queue_.pop_front();
          }
          queue_.push_back({message, Clock::now()});
          drain();
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { drain(); });
  }

 private:
  struct Pending { Cloud::ConstSharedPtr cloud; Clock::time_point received; };

  void publish_status(const std::string& text) {
    std_msgs::msg::String message;
    message.data = text;
    status_->publish(message);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "%s", text.c_str());
  }

  void bypass(const Cloud& cloud, const std::string& reason) {
    filter_->reset();
    last_pose_time_ = -1.0;
    last_debug_ = -1.0;
    ++bypasses_;
    pending_cloud_->publish(cloud);
    Cloud empty = cloud;
    empty.height = 1; empty.width = 0; empty.row_step = 0; empty.data.clear();
    kept_->publish(empty);
    removed_->publish(empty);
    auto header = cloud.header;
    header.frame_id = world_frame_;
    const auto world_empty = world_cloud({}, header);
    for (const auto& publisher : {map_, free_, scan_, seeds_, static_scan_, dynamic_scan_, pending_scan_}) {
      publisher->publish(world_empty);
    }
    publish_markers({}, header);
    publish_status("reason=" + reason + " bypasses=" + std::to_string(bypasses_) + " pending=unclassified");
  }

  void publish_markers(const std::vector<DetectionCluster>& clusters, const std_msgs::msg::Header& header) {
    visualization_msgs::msg::MarkerArray message;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    message.markers.push_back(clear);
    for (const auto& cluster : clusters) {
      visualization_msgs::msg::Marker box;
      box.header = header;
      box.ns = "dynablox_clusters";
      box.id = cluster.id;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.orientation.w = 1.0;
      const Vector midpoint = (cluster.minimum + cluster.maximum) * 0.5;
      const Vector size = (cluster.maximum - cluster.minimum).cwiseMax(Vector::Constant(0.1));
      box.pose.position.x = midpoint.x(); box.pose.position.y = midpoint.y(); box.pose.position.z = midpoint.z();
      box.scale.x = size.x(); box.scale.y = size.y(); box.scale.z = size.z();
      box.color.r = cluster.dynamic ? 1.0 : 0.2;
      box.color.g = cluster.dynamic ? 0.1 : 0.7;
      box.color.b = 0.2; box.color.a = 0.15;
      message.markers.push_back(box);
      box.ns = "dynablox_labels";
      box.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      box.pose.position.z = cluster.maximum.z() + 0.2;
      box.scale.z = 0.20;
      box.color.a = 1.0;
      box.text = "#" + std::to_string(cluster.id) + " free=" + std::to_string(cluster.seed_points) +
          " track=" + std::to_string(cluster.track_length) + (cluster.dynamic ? " DYNAMIC" : " candidate");
      message.markers.push_back(box);
    }
    markers_->publish(message);
  }

  void process(const Cloud& cloud, const Layout& layout, const Eigen::Isometry3d& begin,
               const Eigen::Isometry3d& end, double queue_ms) {
    const auto start = Clock::now();
    const Eigen::Quaterniond begin_rotation(begin.linear()), end_rotation(end.linear());
    const double duration = layout.end - layout.begin;
    if ((duration > 1e-6 && ((end.translation() - begin.translation()).norm() / duration > max_speed_ ||
         begin_rotation.angularDistance(end_rotation) / duration > max_angular_speed_)) ||
        (last_pose_time_ > 0.0 && layout.end > last_pose_time_ &&
         ((end.translation() - last_pose_.translation()).norm() / (layout.end - last_pose_time_) > max_speed_ ||
          Eigen::Quaterniond(last_pose_.linear()).angularDistance(end_rotation) / (layout.end - last_pose_time_) > max_angular_speed_))) {
      bypass(cloud, "pose_jump");
      return;
    }
    std::vector<Vector> points, origins;
    points.reserve(layout.count); origins.reserve(layout.count);
    for (std::size_t index = 0; index < layout.count; ++index) {
      const double ratio = duration > 1e-6 ? std::clamp((layout.time(cloud, index) - layout.begin) / duration, 0.0, 1.0) : 0.0;
      const Vector origin = begin.translation() * (1.0 - ratio) + end.translation() * ratio;
      origins.push_back(origin);
      points.push_back(begin_rotation.slerp(ratio, end_rotation) * layout.point(cloud, index) + origin);
    }
    const auto result = filter_->update(points, end, layout.end);
    last_pose_ = end;
    last_pose_time_ = layout.end;
    kept_->publish(subset(cloud, layout, result.labels, PointLabel::Static));
    removed_->publish(subset(cloud, layout, result.labels, PointLabel::Dynamic));
    pending_cloud_->publish(subset(cloud, layout, result.labels, PointLabel::Unobserved));
    auto header = cloud.header;
    header.frame_id = world_frame_;
    header.stamp = rclcpp::Time(static_cast<int64_t>(layout.end * 1e9));
    scan_->publish(world_cloud(points, header));
    std::vector<Vector> stable, moving, pending;
    for (std::size_t index = 0; index < points.size(); ++index) {
      if (result.labels[index] == PointLabel::Static) stable.push_back(points[index]);
      else if (result.labels[index] == PointLabel::Dynamic) moving.push_back(points[index]);
      else pending.push_back(points[index]);
    }
    static_scan_->publish(world_cloud(stable, header));
    dynamic_scan_->publish(world_cloud(moving, header));
    pending_scan_->publish(world_cloud(pending, header));
    seeds_->publish(world_cloud(result.seeds, header));
    publish_markers(result.clusters, header);
    std::size_t map_points = 0;
    if (last_debug_ < 0.0 || layout.end - last_debug_ >= 0.5) {
      const auto map = filter_->static_map();
      map_points = map.size();
      map_->publish(world_cloud(map, header));
      if (free_->get_subscription_count() > 0) free_->publish(world_cloud(filter_->free_centers(30000), header));
      last_debug_ = layout.end;
    }
    const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    std::ostringstream status;
    status << "reason=" << result.reason << " input=" << points.size() << " static=" << stable.size()
           << " dynamic=" << moving.size() << " pending=" << pending.size() << " seeds=" << result.seeds.size()
           << " blocks=" << result.blocks << " frame=" << result.frame << " clusters=" << result.clusters.size()
           << " map_published_points=" << map_points << " processing_ms=" << elapsed << " queue_ms=" << queue_ms
           << " bypasses=" << bypasses_;
    publish_status(status.str());
  }

  void drain() {
    while (!queue_.empty()) {
      const auto cloud = queue_.front().cloud;
      const double waited = std::chrono::duration<double>(Clock::now() - queue_.front().received).count();
      try {
        if (cloud->header.frame_id != lidar_frame_) throw std::invalid_argument("unexpected_frame");
        const Layout layout(*cloud);
        const auto begin_time = rclcpp::Time(static_cast<int64_t>(layout.begin * 1e9), RCL_ROS_TIME);
        const auto end_time = rclcpp::Time(static_cast<int64_t>(layout.end * 1e9), RCL_ROS_TIME);
        if (!buffer_->canTransform(world_frame_, lidar_frame_, begin_time) ||
            !buffer_->canTransform(world_frame_, lidar_frame_, end_time)) {
          if (waited < tf_wait_) return;
          bypass(*cloud, "tf_timeout");
        } else {
          process(*cloud, layout, pose(buffer_->lookupTransform(world_frame_, lidar_frame_, begin_time)),
                  pose(buffer_->lookupTransform(world_frame_, lidar_frame_, end_time)), waited * 1000.0);
        }
      } catch (const std::exception& error) {
        bypass(*cloud, std::string("invalid_scan:") + error.what());
      }
      queue_.pop_front();
    }
  }

  std::string world_frame_, lidar_frame_;
  double tf_wait_, max_speed_, max_angular_speed_;
  double last_received_ = -1.0, last_pose_time_ = -1.0, last_debug_ = -1.0;
  std::size_t bypasses_ = 0;
  Eigen::Isometry3d last_pose_ = Eigen::Isometry3d::Identity();
  std::unique_ptr<DynabloxAdapter> filter_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::deque<Pending> queue_;
  rclcpp::Subscription<Cloud>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Cloud>::SharedPtr kept_, removed_, pending_cloud_, scan_, static_scan_, dynamic_scan_, pending_scan_;
  rclcpp::Publisher<Cloud>::SharedPtr seeds_, map_, free_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
};
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dynablox_ros2::DynabloxNode>());
  rclcpp::shutdown();
  return 0;
}
