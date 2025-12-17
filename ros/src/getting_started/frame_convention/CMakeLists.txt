cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)

project(frame_convention_example VERSION 1.0.0 LANGUAGES CXX)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(creos_sdk_msgs REQUIRED)
find_package(ros_frame_convention REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp nav_msgs creos_sdk_msgs ros_frame_convention)

install(TARGETS
${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

ament_package()
