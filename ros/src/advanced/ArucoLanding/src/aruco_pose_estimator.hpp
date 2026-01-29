#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <deque>
#include <utility>
#include <memory>

namespace aruco_landing {

class ArucoPoseEstimator {
public:
  explicit ArucoPoseEstimator(rclcpp::Node::SharedPtr node);
  ~ArucoPoseEstimator();
  bool estimate(const cv::Mat& image, geometry_msgs::msg::Pose& pose_out);

private:
  rclcpp::Node::SharedPtr node_;
  cv::Mat camera_matrix, dist_coeffs;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  std::deque<std::pair<rclcpp::Time, geometry_msgs::msg::Pose>> buf;
};

}
