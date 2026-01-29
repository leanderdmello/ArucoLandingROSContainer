#include "landing_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace aruco_landing {

static inline tf2::Quaternion q_from_msg(const geometry_msgs::msg::Quaternion& q) {
  return tf2::Quaternion(q.x, q.y, q.z, q.w);
}
static inline geometry_msgs::msg::Quaternion q_to_msg(const tf2::Quaternion& q) {
  geometry_msgs::msg::Quaternion m; m.x=q.x(); m.y=q.y(); m.z=q.z(); m.w=q.w(); return m;
}

LandingController::LandingController(std::shared_ptr<void>, rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
  reference_pub_ = node_->create_publisher<creos_sdk_msgs::msg::StateReference>("/robot/cmd_state_ref", rclcpp::QoS(10));

  double rate_hz = node_->declare_parameter<double>("controller_rate_hz", 10.0);
  land_on_trigger_ = node_->declare_parameter<bool>("land_on_trigger", true);
  disarm_on_land_  = node_->declare_parameter<bool>("disarm_on_land", true);
  disarm_delay_ms_ = node_->declare_parameter<int>("disarm_delay_ms", 2000);
  xy_tol_          = node_->declare_parameter<double>("xy_tolerance_m", 0.18);
  z_trigger_       = node_->declare_parameter<double>("land_trigger_height_m", 1.0);
  z_tol_           = node_->declare_parameter<double>("z_tolerance_m", 0.10);
  trigger_confirm_needed_ = node_->declare_parameter<int>("trigger_confirm_cycles", 1);
  land_max_retries_ = node_->declare_parameter<int>("land_max_retries", 6);
  land_retry_ms_    = node_->declare_parameter<int>("land_retry_ms", 400);

  std::string send_cmd_srv = node_->declare_parameter<std::string>("send_command_service", "/robot/send_command");
  std::string disarm_topic = node_->declare_parameter<std::string>("disarm_topic", "/robot/cmd_disarm");
  send_command_client_ = node_->create_client<creos_sdk_msgs::srv::SendCommand>(send_cmd_srv);
  disarm_pub_ = node_->create_publisher<std_msgs::msg::Empty>(disarm_topic, rclcpp::QoS(1));

  timer_ = node_->create_wall_timer(
      std::chrono::milliseconds((int)(1000.0/std::max(1.0,rate_hz))),
      std::bind(&LandingController::send_next_waypoint, this));
}

void LandingController::set_waypoints(const std::vector<creos_sdk_msgs::msg::StateReference>& wps){
  std::lock_guard<std::mutex> lk(mtx_);
  waypoints_ = wps;
  current_wp_index_ = 0;
}

void LandingController::update_marker_pose(const geometry_msgs::msg::Pose& p){
  std::lock_guard<std::mutex> lk(mtx_);
  latest_marker_pose_ = p;
  have_marker_ = true;
}

void LandingController::update_internal_pose(const geometry_msgs::msg::Pose& p){
  std::lock_guard<std::mutex> lk(mtx_);
  latest_internal_pose_ = p;
  have_internal_ = true;
}

void LandingController::set_active(bool on){
  active_ = on;
}

void LandingController::send_land() {
  if (land_sent_) return;
  if (!send_command_client_->service_is_ready()) {
    send_command_client_->wait_for_service(std::chrono::seconds(1));
  }
  auto send_once = [this](){
    auto req = std::make_shared<creos_sdk_msgs::srv::SendCommand::Request>();
    req->action = creos_sdk_msgs::srv::SendCommand::Request::LAND;
    (void)send_command_client_->async_send_request(req);
  };
  send_once();
  land_sent_ = true;

  land_retries_left_ = land_max_retries_;
  land_retry_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(land_retry_ms_),
    [this, send_once](){
      if (land_retries_left_-- > 0) send_once();
      else { land_retry_timer_->cancel(); land_retry_timer_.reset(); }
    });

  if (disarm_on_land_ && !disarm_sent_ && disarm_delay_ms_ > 0 && !disarm_timer_) {
    disarm_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(disarm_delay_ms_),
      [this]() {
        this->send_disarm();
        disarm_timer_->cancel();
        disarm_timer_.reset();
      });
  }
}

void LandingController::send_disarm() {
  if (disarm_sent_) return;
  std_msgs::msg::Empty msg;
  disarm_pub_->publish(msg);
  disarm_sent_ = true;
}

void LandingController::send_next_waypoint()
{
  if (!active_) return;

  geometry_msgs::msg::Pose marker, internal;
  creos_sdk_msgs::msg::StateReference ref_marker;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_marker_ || !have_internal_) return;
    if (current_wp_index_ >= waypoints_.size()) return;
    marker   = latest_marker_pose_;
    internal = latest_internal_pose_;
    ref_marker = waypoints_[current_wp_index_];
  }

  geometry_msgs::msg::Point delta;
  delta.x = ref_marker.pose.position.x - marker.position.x;
  delta.y = ref_marker.pose.position.y - marker.position.y;
  delta.z = ref_marker.pose.position.z - marker.position.z;

  creos_sdk_msgs::msg::StateReference ref_out = ref_marker;
  ref_out.header.stamp = node_->now();
  ref_out.header.frame_id = "NED_odom";
  ref_out.translation_mode = creos_sdk_msgs::msg::StateReference::TRANSLATION_MODE_POSITION;
  ref_out.orientation_mode = creos_sdk_msgs::msg::StateReference::ORIENTATION_MODE_ATTITUDE;

  ref_out.pose.position.x = internal.position.x + delta.x;
  ref_out.pose.position.y = internal.position.y + delta.y;
  ref_out.pose.position.z = internal.position.z + delta.z;

  tf2::Quaternion q_int = q_from_msg(internal.orientation);
  ref_out.pose.orientation = q_to_msg(q_int);

  reference_pub_->publish(ref_out);

  bool near_xy = std::fabs(marker.position.x) <= xy_tol_ && std::fabs(marker.position.y) <= xy_tol_;
  double height_m = std::fabs(marker.position.z);
  bool below_or_near = height_m <= (z_trigger_ + z_tol_);

  if (near_xy && below_or_near) ++trigger_count_; else trigger_count_ = 0;

  if (land_on_trigger_ && !land_sent_ && trigger_count_ >= trigger_confirm_needed_) {
    send_land();
    active_ = false;
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    current_wp_index_++;
  }
}

}
