cmake_minimum_required(VERSION 3.5)
project(bitbots_odometry)

# Add support for C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

find_package(ament_cmake REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(bitbots_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rot_conv REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

generate_parameter_library(
  odometry_parameters
  config/odometry_config_template.yaml
)

include_directories(include)

add_compile_options(-Wall -Werror -Wno-unused)

add_executable(odometry_fuser src/odometry_fuser.cpp)
add_executable(motion_odometry src/motion_odometry.cpp)
add_executable(walk_support_state_detector src/walk_support_state_detector.cpp)

target_link_libraries(
  motion_odometry
  rclcpp::rclcpp
  odometry_parameters
)

target_link_libraries(
  walk_support_state_detector
  rclcpp::rclcpp
)

## Specify libraries to link a library or executable target against
ament_target_dependencies(
  motion_odometry
  ament_cmake
  biped_interfaces
  bitbots_docs
  bitbots_msgs
  bitbots_utils
  generate_parameter_library
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  rot_conv
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(
  walk_support_state_detector
  bitbots_msgs
  biped_interfaces
)

ament_target_dependencies(
  odometry_fuser
  ament_cmake
  biped_interfaces
  bitbots_docs
  bitbots_utils
  Eigen3
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  rot_conv
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

enable_bitbots_docs()

install(TARGETS motion_odometry
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS odometry_fuser
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS walk_support_state_detector
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

ament_package()
