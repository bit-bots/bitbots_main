cmake_minimum_required(VERSION 3.8)
project(wolfgang_ik)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Build with release optimizations and debug symbols by default
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebug)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(backward_ros REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)

add_compile_options(-Wall -Werror -Wno-unused -Wfatal-errors)

set(INCLUDE_DIRS include)
include_directories(${INCLUDE_DIRS})

add_executable(ik src/main.cpp )
add_executable(node src/ros.cpp)

ament_target_dependencies(
  ik
  ament_cmake
  ament_index_cpp
  bitbots_docs
  tf2_eigen
  Eigen3)

ament_target_dependencies(
  node
  ament_cmake
  ament_index_cpp
  bitbots_docs
  tf2_eigen
  sensor_msgs
  Eigen3)



enable_bitbots_docs()

install(TARGETS ik DESTINATION lib/${PROJECT_NAME})
install(TARGETS node DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
