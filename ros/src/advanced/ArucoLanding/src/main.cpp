#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <creos_sdk_msgs/msg/state.hpp>
#include <creos_sdk_msgs/msg/control_source.hpp>
#include <creos_sdk_msgs/msg/state_reference.hpp>
#include "common/logging.hpp"
#include "common/drone_state.hpp"
#include "common/remote_controller_interface.hpp"
#include "aruco_pose_estimator.hpp"
#include "landing_planner.hpp"
#include "landing_controller.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<rclcpp::Node>("aruco_landing_main");
    setup_logging("aruco_landing", node->get_logger());

    RCLCPP_WARN(node->get_logger(), "-------------------------------------------------------------------------------");
    RCLCPP_WARN(node->get_logger(), "-------------------------------------------------------------------------------");
    RCLCPP_WARN(node->get_logger(), "------  When testing for the first time, always remove the propellers!!  ------");
    RCLCPP_WARN(node->get_logger(), "-------------------------------------------------------------------------------");
    RCLCPP_WARN(node->get_logger(), "-------------------------------------------------------------------------------");
    RCLCPP_INFO(node->get_logger(), "Running: aruco_landing");

    node->declare_parameter<std::string>("controller", "jeti");
    const std::string controller_type = node->get_parameter("controller").as_string();

    auto drone_state = std::make_shared<DroneState>();

    auto sdq = rclcpp::SensorDataQoS();

    auto global_pose_sub =
        node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/robot/pose", sdq, drone_state->GetGlobalPoseCallback());
    auto state_sub =
        node->create_subscription<creos_sdk_msgs::msg::State>(
            "/robot/state", sdq, drone_state->GetStateCallback());
    auto control_source_sub =
        node->create_subscription<creos_sdk_msgs::msg::ControlSource>(
            "/robot/current_control_source", sdq, drone_state->GetControlSourceCallback());

    std::shared_ptr<IRemoteController> controller =
        CreateRemoteController(controller_type == "herelink" ? ControllerType::kHerelink
                                                             : ControllerType::kJeti,
                               node->get_logger());
    RCLCPP_INFO(node->get_logger(), "Remote controller: %s", controller_type.c_str());
    RCLCPP_INFO(node->get_logger(), "JetiRemote: Use the SF switch to activate the execution of the example");

    bool execution_active = node->declare_parameter<bool>("auto_execute_on_start", false);
    RCLCPP_INFO(node->get_logger(), "Execution active (on start): %s", execution_active ? "true" : "false");

    auto controller_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        "/robot/joy", sdq, controller->GetControllerStateCallback());
    controller->RegisterActivationButtonCallback([&]() {
      execution_active = !execution_active;
      RCLCPP_INFO(node->get_logger(), "Execution active: %s", execution_active ? "true" : "false");
    });

    auto landing_controller = std::make_shared<aruco_landing::LandingController>(drone_state, node);
    RCLCPP_INFO(node->get_logger(), "LandingController ready");

    aruco_landing::ArucoPoseEstimator estimator(node);
    RCLCPP_INFO(node->get_logger(), "Done aruco setup");

    auto internal_pose_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/robot/pose", sdq,
      [landing_controller, &node](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m){
        landing_controller->update_internal_pose(m->pose.pose);
      });

    auto qos_aruco = rclcpp::QoS(rclcpp::KeepLast(10));
    auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/aruco_landing/estimated_pose", qos_aruco,
        [landing_controller, &execution_active, &node]
        (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
          landing_controller->update_marker_pose(msg->pose.pose);

          if (!execution_active) {
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                                 "Execution inactive; skipping waypoint generation");
            return;
          }

          auto waypoints = aruco_landing::generate_landing_path(msg->pose.pose);
          RCLCPP_INFO(node->get_logger(), "Generated %zu waypoint(s) toward marker origin", waypoints.size());
          landing_controller->set_waypoints(waypoints);
          landing_controller->set_active(true);
        });

    RCLCPP_INFO(node->get_logger(), "Done landing controller");
    rclcpp::spin(node);

    pose_sub.reset();
    internal_pose_sub.reset();
    controller_sub.reset();
    control_source_sub.reset();
    state_sub.reset();
    global_pose_sub.reset();
  }
  rclcpp::shutdown();
  return 0;
}
