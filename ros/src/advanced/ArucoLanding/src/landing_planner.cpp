#include "landing_planner.hpp"
#include <creos_sdk_msgs/msg/state_reference.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath>
#include <vector>

namespace aruco_landing {

static constexpr double XY_MAX_STEP = 0.02;
static constexpr double DZ_MAX = 0.02;
static constexpr double Z_TOUCHDOWN = 0.15;

static inline double move_toward(double cur, double target, double max_step) {
  if (max_step <= 0.0) return cur;
  double d = target - cur;
  if (std::fabs(d) <= max_step) return target;
  return cur + std::copysign(max_step, d);
}

std::vector<creos_sdk_msgs::msg::StateReference>
generate_landing_path(const geometry_msgs::msg::Pose& current_pose)
{
  double x = current_pose.position.x, y = current_pose.position.y, z = current_pose.position.z;

  double x_next = x, y_next = y;
  double r = std::hypot(x, y);
  if (r > 1e-6) {
    double r_after = std::max(0.0, r - XY_MAX_STEP);
    double s = r_after / r;
    x_next = x * s;
    y_next = y * s;
  } else {
    x_next = 0.0; y_next = 0.0;
  }

  double z_next = move_toward(z, Z_TOUCHDOWN, DZ_MAX);

  creos_sdk_msgs::msg::StateReference ref;
  ref.pose.position.x = x_next;
  ref.pose.position.y = y_next;
  ref.pose.position.z = z_next;
  ref.pose.orientation = current_pose.orientation;
  ref.translation_mode = creos_sdk_msgs::msg::StateReference::TRANSLATION_MODE_POSITION;
  ref.orientation_mode = creos_sdk_msgs::msg::StateReference::ORIENTATION_MODE_ATTITUDE;
  return {ref};
}

}
