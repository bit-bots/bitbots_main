cmake_minimum_required(VERSION 3.8)
project(bitbots_head_mover)


# Build with release optimizations and debug symbols by default
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebug)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(bio_ik REQUIRED)
find_package(bio_ik_msgs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)
generate_parameter_library(
  head_parameters # cmake target name for the parameter library
  config/head_config.yml)

add_executable(move_head src/move_head.cpp)
target_link_libraries(move_head rclcpp::rclcpp head_parameters)

ament_target_dependencies(
  move_head
  backward_ros
  bio_ik
  bio_ik_msgs
  bitbots_msgs
  generate_parameter_library
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros)

install(TARGETS move_head DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
