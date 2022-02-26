cmake_minimum_required(VERSION 3.5)
project(bitbots_convenience_frames)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)

add_compile_options(-Wall -Werror -Wno-unused)

add_executable(convenience_frames src/convenience_frames.cpp)

ament_target_dependencies(convenience_frames
  ament_cmake
  bitbots_docs
  bitbots_msgs
  humanoid_league_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

enable_bitbots_docs()

install(TARGETS convenience_frames
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
