cmake_minimum_required(VERSION 3.5)
project(bitbots_booster)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Build with release optimizations and debug symbols by default
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebug)
endif()

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(booster_msgs REQUIRED)
find_package(booster_interface REQUIRED)

add_compile_options(-Wall -Werror -Wno-unused)

add_executable(booster src/main.cpp)

ament_target_dependencies(
  booster
  ament_cmake
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  booster_msgs
  booster_interface)

target_include_directories(booster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS booster DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
