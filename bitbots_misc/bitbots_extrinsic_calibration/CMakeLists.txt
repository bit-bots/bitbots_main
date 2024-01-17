cmake_minimum_required(VERSION 3.5)
project(bitbots_extrinsic_calibration)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rot_conv REQUIRED)
find_package(backward_ros REQUIRED)

include_directories(include)

add_compile_options(-Wall -Werror -Wno-unused)

add_executable(extrinsic_calibration src/extrinsic_calibration.cpp)

ament_target_dependencies(extrinsic_calibration
  ament_cmake
  bitbots_docs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  rot_conv
)

enable_bitbots_docs()

install(TARGETS extrinsic_calibration
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

ament_package()
