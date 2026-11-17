#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class Map3DTo2D : public rclcpp::Node
{
public:
  Map3DTo2D()
  : Node("map_3d_to_2d")
  {
    input_topic_ = declare_parameter("input_topic", std::string("/zhao_lio/map"));
    output_topic_ = declare_parameter("output_topic", std::string("/map2d"));
    resolution_ = declare_parameter("resolution", 0.05);
    z_min_ = declare_parameter("z_min", 0.1);
    z_max_ = declare_parameter("z_max", 1.5);
    ground_z_ = declare_parameter("ground_z", 0.0);
    z_min_topic_ = declare_parameter("z_min_topic", std::string("/map_3d_to_2d/z_min"));
    z_max_topic_ = declare_parameter("z_max_topic", std::string("/map_3d_to_2d/z_max"));
    occupancy_threshold_ = declare_parameter("occupancy_threshold", 1);
    frame_id_ = declare_parameter("frame_id", std::string());
    padding_ = declare_parameter("padding", 1.0);
    max_range_ = declare_parameter("max_range", 0.0);
    if (!std::isfinite(resolution_) || resolution_ < 0.01 ||
      !std::isfinite(z_min_) || !std::isfinite(z_max_) || z_min_ > z_max_ ||
      !std::isfinite(ground_z_) || !std::isfinite(padding_) || padding_ < 0.0 ||
      !std::isfinite(max_range_) || max_range_ < 0.0 ||
      occupancy_threshold_ < 1 || occupancy_threshold_ > 65535)
    {
      throw std::invalid_argument("Invalid height range, resolution, padding, range or threshold");
    }

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, map_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { on_cloud(*message); });
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, map_qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      output_topic_ + "/markers", map_qos);

    const auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    range_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      "/map_3d_to_2d/range", control_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/map_3d_to_2d/status", control_qos);
    range_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/map_3d_to_2d/set_range", rclcpp::QoS(1).reliable(),
      [this](geometry_msgs::msg::Vector3::ConstSharedPtr message) {
        set_z_range(message->x, message->y);
      });
    z_min_sub_ = create_subscription<std_msgs::msg::Float32>(
      z_min_topic_, control_qos,
      [this](std_msgs::msg::Float32::ConstSharedPtr message) {
        set_z_range(static_cast<double>(message->data), z_max_);
      });
    z_max_sub_ = create_subscription<std_msgs::msg::Float32>(
      z_max_topic_, control_qos,
      [this](std_msgs::msg::Float32::ConstSharedPtr message) {
        set_z_range(z_min_, static_cast<double>(message->data));
      });

    publish_range();
    publish_status("Waiting for point-cloud map: " + input_topic_);
    RCLCPP_INFO(get_logger(), "map tool ready: input=%s output=%s z=[%.2f, %.2f]",
      input_topic_.c_str(), output_topic_.c_str(), z_min_, z_max_);
  }

private:
  void publish_range()
  {
    geometry_msgs::msg::Vector3 range;
    range.x = z_min_;
    range.y = z_max_;
    range.z = ground_z_;
    range_pub_->publish(range);
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    status_pub_->publish(message);
  }

  void publish_markers(const nav_msgs::msg::OccupancyGrid & map)
  {
    visualization_msgs::msg::Marker background;
    background.header = map.header;
    background.ns = "height_grid";
    background.id = 0;
    background.type = visualization_msgs::msg::Marker::CUBE;
    background.action = visualization_msgs::msg::Marker::ADD;
    background.pose = map.info.origin;
    background.scale.x = map.info.width * map.info.resolution;
    background.scale.y = map.info.height * map.info.resolution;
    background.scale.z = 0.005;
    background.pose.position.x += background.scale.x / 2.0;
    background.pose.position.y += background.scale.y / 2.0;
    background.pose.position.z -= 0.015;
    background.color.r = background.color.g = background.color.b = 0.65F;
    background.color.a = 1.0F;

    visualization_msgs::msg::Marker occupied;
    occupied.header = map.header;
    occupied.ns = "height_grid";
    occupied.id = 1;
    occupied.type = visualization_msgs::msg::Marker::CUBE_LIST;
    occupied.action = visualization_msgs::msg::Marker::ADD;
    occupied.pose = map.info.origin;
    occupied.scale.x = occupied.scale.y = map.info.resolution;
    occupied.scale.z = 0.01;
    occupied.color.r = occupied.color.g = occupied.color.b = 0.02F;
    occupied.color.a = 1.0F;
    for (size_t index = 0; index < map.data.size(); ++index) {
      if (map.data[index] != 100) {
        continue;
      }
      geometry_msgs::msg::Point point;
      point.x = (index % map.info.width + 0.5) * map.info.resolution;
      point.y = (index / map.info.width + 0.5) * map.info.resolution;
      occupied.points.push_back(point);
    }
    if (occupied.points.empty()) {
      occupied.action = visualization_msgs::msg::Marker::DELETE;
    }
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(std::move(background));
    markers.markers.push_back(std::move(occupied));
    markers_pub_->publish(markers);
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2 & message)
  {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    try {
      pcl::fromROSMsg(message, *cloud);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "cannot convert PointCloud2: %s", error.what());
      return;
    }

    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    filtered->reserve(cloud->size());
    for (const auto & point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (max_range_ > 0.0 && std::hypot(point.x, point.y) > max_range_) {
        continue;
      }
      filtered->push_back(point);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      cloud_ = std::move(filtered);
      stamp_ = message.header.stamp;
      input_frame_id_ = message.header.frame_id;
    }
    publish_map();
  }

  void set_z_range(double candidate_min, double candidate_max)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(candidate_min) || !std::isfinite(candidate_max) || candidate_min > candidate_max) {
      RCLCPP_WARN(get_logger(), "ignoring invalid z range [%.3f, %.3f]", candidate_min, candidate_max);
      return;
    }
    z_min_ = candidate_min;
    z_max_ = candidate_max;
    publish_range();
    publish_map_locked();
  }

  void publish_map()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publish_map_locked();
  }

  void publish_map_locked()
  {
    if (!cloud_) {
      return;
    }

    const double resolution = std::max(resolution_, 0.01);
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    size_t selected = 0;
    for (const auto & point : cloud_->points) {
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
      if (point.z >= ground_z_ + z_min_ && point.z <= ground_z_ + z_max_) {
        ++selected;
      }
    }
    if (cloud_->empty()) {
      min_x = min_y = max_x = max_y = 0.0;
    }

    min_x = std::floor((min_x - padding_) / resolution) * resolution;
    min_y = std::floor((min_y - padding_) / resolution) * resolution;
    max_x += padding_;
    max_y += padding_;
    const double columns = std::ceil((max_x - min_x) / resolution) + 1;
    const double rows = std::ceil((max_y - min_y) / resolution) + 1;
    if (!std::isfinite(columns * rows) || columns * rows > 16000000.0) {
      publish_status("Map exceeds 16 million cells; increase resolution in config");
      RCLCPP_ERROR(get_logger(), "Map too large; increase resolution");
      return;
    }
    const auto width = static_cast<uint32_t>(columns);
    const auto height = static_cast<uint32_t>(rows);
    const uint64_t cell_count = static_cast<uint64_t>(width) * height;
    if (width == 0 || height == 0 || cell_count > 16000000ULL) {
      RCLCPP_WARN(get_logger(), "map size %ux%u is invalid or too large", width, height);
      return;
    }

    std::vector<uint16_t> counts(cell_count, 0);
    for (const auto & point : cloud_->points) {
      if (point.z < ground_z_ + z_min_ || point.z > ground_z_ + z_max_) {
        continue;
      }
      const auto ix = static_cast<int>(std::floor((point.x - min_x) / resolution));
      const auto iy = static_cast<int>(std::floor((point.y - min_y) / resolution));
      if (ix >= 0 && iy >= 0 && ix < static_cast<int>(width) && iy < static_cast<int>(height)) {
        auto & count = counts[static_cast<size_t>(iy) * width + static_cast<size_t>(ix)];
        if (count < std::numeric_limits<uint16_t>::max()) {
          ++count;
        }
      }
    }

    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp = stamp_;
    map.header.frame_id = frame_id_.empty() ? input_frame_id_ : frame_id_;
    map.info.resolution = static_cast<float>(resolution);
    map.info.width = width;
    map.info.height = height;
    map.info.origin.position.x = min_x;
    map.info.origin.position.y = min_y;
    map.info.origin.position.z = ground_z_;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(cell_count, -1);
    for (size_t index = 0; index < counts.size(); ++index) {
      if (counts[index] >= static_cast<uint16_t>(std::max(1, occupancy_threshold_))) {
        map.data[index] = 100;
      }
    }
    map_pub_->publish(map);
    publish_markers(map);
    publish_status("Cloud=" + std::to_string(cloud_->size()) +
      " | selected=" + std::to_string(selected) +
      " | occupied=" + std::to_string(std::count(map.data.begin(), map.data.end(), 100)) +
      " | grid=" + std::to_string(width) + "x" + std::to_string(height) +
      (selected == 0 ? " | No points in height range" : ""));
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "z=[%.2f, %.2f], selected=%zu, grid=%ux%u", z_min_, z_max_, selected, width, height);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string z_min_topic_;
  std::string z_max_topic_;
  std::string frame_id_;
  double resolution_{0.05};
  double z_min_{0.1};
  double z_max_{1.5};
  double ground_z_{0.0};
  int occupancy_threshold_{1};
  double padding_{1.0};
  double max_range_{0.0};
  std::mutex mutex_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_;
  builtin_interfaces::msg::Time stamp_;
  std::string input_frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr z_min_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr z_max_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr range_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr range_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Map3DTo2D>());
  rclcpp::shutdown();
  return 0;
}
