cmake_minimum_required(VERSION 3.5)
project(bitbots_odometry)

# Add support for C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(bitbots_docs REQUIRED)
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
  config/odometry_config.yml
)

include_directories(include)

add_compile_options(-Wall -Werror -Wno-unused)

add_executable(odometry_fuser src/odometry_fuser.cpp)
add_executable(motion_odometry src/motion_odometry.cpp)

target_link_libraries(
  motion_odometry
  rclcpp::rclcpp
  odometry_parameters
)

## Specify libraries to link a library or executable target against
ament_target_dependencies(
  motion_odometry
  tf2_eigen
  tf2
  nav_msgs
  message_filters
  tf2_geometry_msgs
  biped_interfaces
  geometry_msgs
  bitbots_docs
  rot_conv
  sensor_msgs
  tf2_ros
  ament_cmake
  generate_parameter_library
  rclcpp
)

ament_target_dependencies(
  odometry_fuser
  tf2_eigen
  tf2
  nav_msgs
  message_filters
  tf2_geometry_msgs
  biped_interfaces
  geometry_msgs
  bitbots_docs
  rot_conv
  sensor_msgs
  tf2_ros
  ament_cmake
  rclcpp
  Eigen3
)

enable_bitbots_docs()

install(TARGETS motion_odometry
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS odometry_fuser
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
