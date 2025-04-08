cmake_minimum_required(VERSION 3.5)
project(bitbots_moveit_bindings)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Build with release optimizations and debug symbols by default
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebug)
endif()

set(PYBIND11_PYTHON_VERSION 3)
set(PYBIND11_FINDPYTHON ON)
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(bio_ik REQUIRED)
find_package(bio_ik_msgs REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(rclcpp REQUIRED)
find_package(ros2_python_extension)
find_package(tf2 REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_ros REQUIRED)

add_compile_options(-Wall -Wno-unused)

enable_bitbots_docs()

pybind11_add_module(libbitbots_moveit_bindings SHARED
                    src/bitbots_moveit_bindings.cpp)

ament_target_dependencies(
  libbitbots_moveit_bindings
  PUBLIC
  backward_ros
  bio_ik
  bio_ik_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  rclcpp
  ros2_python_extension
  tf2
  tf2_ros
  tf2_kdl)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS libbitbots_moveit_bindings
        DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

ament_package()
