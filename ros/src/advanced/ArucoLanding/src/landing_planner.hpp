#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <creos_sdk_msgs/msg/state_reference.hpp>
#include <vector>

namespace aruco_landing {

std::vector<creos_sdk_msgs::msg::StateReference>
generate_landing_path(const geometry_msgs::msg::Pose& current_pose);

}
