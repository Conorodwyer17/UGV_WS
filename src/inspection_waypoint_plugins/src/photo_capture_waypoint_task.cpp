// Copyright 2026 inspection_waypoint_plugins
// SPDX-License-Identifier: Apache-2.0

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace inspection_waypoint_plugins
{

class PhotoCaptureWaypointTask : public nav2_core::WaypointTaskExecutor
{
public:
  PhotoCaptureWaypointTask() = default;
  ~PhotoCaptureWaypointTask() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override
  {
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error{"PhotoCaptureWaypointTask: failed to lock parent node"};
    }
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name + ".enabled", rclcpp::ParameterValue(true));
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name + ".capture_photo_topic",
      rclcpp::ParameterValue(std::string("/inspection_manager/capture_photo")));
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name + ".capture_result_topic",
      rclcpp::ParameterValue(std::string("/inspection_manager/capture_result")));
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name + ".timeout_s", rclcpp::ParameterValue(15.0));

    node->get_parameter(plugin_name + ".enabled", enabled_);
    node->get_parameter(plugin_name + ".capture_photo_topic", capture_photo_topic_);
    node->get_parameter(plugin_name + ".capture_result_topic", capture_result_topic_);
    node->get_parameter(plugin_name + ".timeout_s", timeout_s_);

    RCLCPP_INFO(
      logger_,
      "PhotoCaptureWaypointTask: enabled=%s topics capture=%s result=%s timeout=%.1fs",
      enabled_ ? "true" : "false",
      capture_photo_topic_.c_str(),
      capture_result_topic_.c_str(),
      timeout_s_);
  }

  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
    const int & curr_waypoint_index) override
  {
    if (!enabled_) {
      return true;
    }
    RCLCPP_INFO(
      logger_, "PhotoCaptureWaypointTask: waypoint index %i — triggering capture",
      curr_waypoint_index);

    // Dedicated helper node + executor so we do not deadlock the waypoint_follower executor
    // while waiting for capture_result.
    auto helper = std::make_shared<rclcpp::Node>(
      "photo_capture_wp_" + std::to_string(curr_waypoint_index));
    std::atomic<bool> done{false};
    std::atomic<bool> ok{false};

    auto sub = helper->create_subscription<std_msgs::msg::String>(
      capture_result_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [&done, &ok](const std_msgs::msg::String::SharedPtr msg) {
        done = true;
        ok = (msg->data.size() >= 7 && msg->data.compare(0, 7, "SUCCESS") == 0);
      });

    auto pub = helper->create_publisher<std_msgs::msg::Bool>(capture_photo_topic_, 10);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(helper);

    // Allow discovery to match publishers/subscribers
    const auto t0 = std::chrono::steady_clock::now();
    while (rclcpp::ok() &&
      std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(500))
    {
      exec.spin_some(std::chrono::milliseconds(20));
    }

    std_msgs::msg::Bool trig;
    trig.data = true;
    pub->publish(trig);

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s_);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (done) {
        break;
      }
      exec.spin_some(std::chrono::milliseconds(20));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    exec.remove_node(helper);
    helper.reset();

    if (!done) {
      RCLCPP_ERROR(logger_, "PhotoCaptureWaypointTask: timeout waiting for capture_result");
      return false;
    }
    if (!ok) {
      RCLCPP_WARN(logger_, "PhotoCaptureWaypointTask: capture reported FAILURE");
    }
    return ok;
  }

private:
  rclcpp::Logger logger_{rclcpp::get_logger("photo_capture_waypoint_task")};
  rclcpp::Clock::SharedPtr clock_;
  bool enabled_{true};
  std::string capture_photo_topic_;
  std::string capture_result_topic_;
  double timeout_s_{15.0};
};

}  // namespace inspection_waypoint_plugins

PLUGINLIB_EXPORT_CLASS(
  inspection_waypoint_plugins::PhotoCaptureWaypointTask,
  nav2_core::WaypointTaskExecutor)
