#pragma once
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <mutex>
#include <geometry_msgs/msg/pose.hpp>
#include <creos_sdk_msgs/msg/state_reference.hpp>
#include <creos_sdk_msgs/srv/send_command.hpp>
#include <std_msgs/msg/empty.hpp>

namespace aruco_landing {

class LandingController {
public:
  LandingController(std::shared_ptr<void> drone_state, rclcpp::Node::SharedPtr node);
  void set_waypoints(const std::vector<creos_sdk_msgs::msg::StateReference>& wps);
  void update_marker_pose(const geometry_msgs::msg::Pose& p);
  void update_internal_pose(const geometry_msgs::msg::Pose& p);
  void set_active(bool on);

private:
  void send_next_waypoint();
  void send_land();
  void send_disarm();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<creos_sdk_msgs::msg::StateReference>::SharedPtr reference_pub_;
  rclcpp::Client<creos_sdk_msgs::srv::SendCommand>::SharedPtr send_command_client_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr disarm_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr disarm_timer_;
  rclcpp::TimerBase::SharedPtr land_retry_timer_;

  std::vector<creos_sdk_msgs::msg::StateReference> waypoints_;
  size_t current_wp_index_{0};
  bool active_{false};
  geometry_msgs::msg::Pose latest_marker_pose_;
  geometry_msgs::msg::Pose latest_internal_pose_;
  bool have_marker_{false};
  bool have_internal_{false};
  std::mutex mtx_;

  bool land_on_trigger_{true};
  bool disarm_on_land_{true};
  int disarm_delay_ms_{2000};
  double xy_tol_{0.18};
  double z_trigger_{1.0};
  double z_tol_{0.10};
  int trigger_confirm_needed_{1};
  int trigger_count_{0};
  bool land_sent_{false};
  bool disarm_sent_{false};
  int land_max_retries_{6};
  int land_retry_ms_{400};
  int land_retries_left_{0};
};

}
