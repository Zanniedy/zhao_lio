#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "zhao_lio/zhao_lio.h"
#include "zhao_lio_backend/backend.h"
#include "zhao_lio_ros2/backend_config.h"

namespace zhao_lio_ros2 {

// Runs the back end on its own thread next to the LIO callback and publishes
//   TF map -> odom, /zhao_lio/backend/pose (map frame), /zhao_lio/backend/path, /zhao_lio/backend/state,
// and offers /zhao_lio/backend/save_map. When the back end asks for a front-end restart,
// reset_requested() turns true until consumed by the LIO thread.
class BackendRunner {
 public:
  BackendRunner(rclcpp::Node& node, const BackendNodeConfig& cfg, const std::string& odom_frame)
      : node_(node), cfg_(cfg), odom_frame_(odom_frame), backend_(cfg.params) {
    backend_.set_log_callback([this](const std::string& m) { RCLCPP_INFO(node_.get_logger(), "[backend] %s", m.c_str()); });
    backend_.set_reset_callback([this] { reset_.store(true); });
    if (!cfg_.map_path.empty()) {
      std::string err;
      if (!backend_.load_map(cfg_.map_path, &err)) throw std::runtime_error(err);
    }
    if (!cfg_.traj_path.empty()) { traj_.open(cfg_.traj_path); traj_ << std::fixed << std::setprecision(9); }
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    pub_pose_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>("/zhao_lio/backend/pose", 100);
    pub_path_ = node_.create_publisher<nav_msgs::msg::Path>("/zhao_lio/backend/path", 10);
    pub_state_ = node_.create_publisher<std_msgs::msg::String>("/zhao_lio/backend/state",
                                                               rclcpp::QoS(1).transient_local());
    srv_save_ = node_.create_service<std_srvs::srv::Trigger>("/zhao_lio/backend/save_map",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          const std::string dir = cfg_.save_dir.empty() ? expand_home("~/zhao_lio_maps/package") : cfg_.save_dir;
          std::string err;
          if (cfg_.params.mode != zhao_lio_backend::Mode::kMapping) {
            res->success = false;
            res->message = "map packages are only saved in mapping mode";
            return;
          }
          std::lock_guard<std::mutex> lk(backend_mutex_);
          res->success = backend_.save_map(dir, &err);
          res->message = res->success ? "map package -> " + dir : err;
        });
    path_.header.frame_id = cfg_.map_frame;
    worker_ = std::thread([this] { run(); });
  }

  ~BackendRunner() {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    // Only mapping builds a complete package; localization drops clouds outside its window and
    // must never overwrite the prior map.
    if (!cfg_.save_dir.empty() && cfg_.params.mode == zhao_lio_backend::Mode::kMapping) {
      std::string err;
      if (!backend_.save_map(cfg_.save_dir, &err)) RCLCPP_WARN(node_.get_logger(), "map package not saved: %s", err.c_str());
    }
  }

  // Called from the LIO thread for every odometry result.
  void push(const zhao_lio::ScanResult& r, int session) {
    Item it;
    it.time = r.time;
    it.T = zhao_lio_backend::pose_from(r.state.rot, r.state.pos);
    it.up = -r.state.grav.normalized();
    it.points = r.points_world;
    it.session = session;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      queue_.push_back(std::move(it));
      // Never let the back end fall far behind: drop the oldest frames.
      while (queue_.size() > 50) { queue_.pop_front(); ++dropped_; }
    }
    cv_.notify_one();
  }

  bool consume_reset() { return reset_.exchange(false); }

 private:
  struct Item {
    double time = 0;
    zhao_lio_backend::Pose T;
    Eigen::Vector3d up;
    std::vector<Eigen::Vector3f> points;
    int session = 0;
  };

  void run() {
    for (;;) {
      Item it;
      {
        std::unique_lock<std::mutex> lk(mutex_);
        cv_.wait(lk, [this] { return stop_ || !queue_.empty(); });
        if (stop_ && queue_.empty()) return;
        it = std::move(queue_.front());
        queue_.pop_front();
      }
      zhao_lio_backend::BackendOutput out;
      {
        std::lock_guard<std::mutex> lk(backend_mutex_);
        out = backend_.push(it.time, it.T, it.points, it.session, it.up);
      }
      publish(it.time, out);
    }
  }

  static void to_msg(const zhao_lio_backend::Pose& T, geometry_msgs::msg::Pose& p) {
    const Eigen::Quaterniond q(T.linear());
    p.position.x = T.translation().x(); p.position.y = T.translation().y(); p.position.z = T.translation().z();
    p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z(); p.orientation.w = q.w();
  }

  void publish(double t, const zhao_lio_backend::BackendOutput& out) {
    const rclcpp::Time stamp(static_cast<int64_t>(t * 1e9));
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = cfg_.map_frame;
    tf.child_frame_id = odom_frame_;
    const Eigen::Quaterniond q(out.T_map_odom.linear());
    tf.transform.translation.x = out.T_map_odom.translation().x();
    tf.transform.translation.y = out.T_map_odom.translation().y();
    tf.transform.translation.z = out.T_map_odom.translation().z();
    tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y(); tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
    tf_->sendTransform(tf);

    if (out.state != last_state_) {
      std_msgs::msg::String s;
      s.data = std::string(zhao_lio_backend::to_string(out.state)) + (out.trusted ? " trusted" : " untrusted");
      pub_state_->publish(s);
      last_state_ = out.state;
    }
    if (!out.trusted) return;  // pose output is frozen at the last trusted value
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = cfg_.map_frame;
    to_msg(out.T_map_body, ps.pose);
    pub_pose_->publish(ps);
    if (out.keyframe) {
      path_.header.stamp = stamp;
      path_.poses.push_back(ps);
      pub_path_->publish(path_);
    }
    if (traj_.is_open()) write_tum(traj_, t, out.T_map_body);
  }

  rclcpp::Node& node_;
  BackendNodeConfig cfg_;
  std::string odom_frame_;
  zhao_lio_backend::Backend backend_;
  std::mutex backend_mutex_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Item> queue_;
  bool stop_ = false;
  std::size_t dropped_ = 0;
  std::atomic<bool> reset_{false};
  std::thread worker_;
  std::ofstream traj_;
  zhao_lio_backend::State last_state_ = zhao_lio_backend::State::kInitializing;
  nav_msgs::msg::Path path_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
};

}  // namespace zhao_lio_ros2
