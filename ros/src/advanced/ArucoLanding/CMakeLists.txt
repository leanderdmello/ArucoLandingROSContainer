cmake_minimum_required(VERSION 3.20)

project(aruco_landing VERSION 1.0.0 LANGUAGES CXX)

option(BUILD_TESTS "Build the tests" OFF)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(advanced_common REQUIRED)
find_package(OpenCV REQUIRED)
find_package(creos_sdk_msgs REQUIRED)

# Build library
add_library(${PROJECT_NAME}_lib STATIC
  src/aruco_pose_estimator.cpp
  src/landing_planner.cpp
  src/landing_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  creos_sdk_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  advanced_common
  OpenCV
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../common/include
  /usr/include/creos_sdk_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
)

install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(${PROJECT_NAME}
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  creos_sdk_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  advanced_common
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

