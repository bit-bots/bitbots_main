cmake_minimum_required(VERSION 3.9)
project(bitbots_quintic_walk)

# Add support for C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Include CheckIPOSupported module
include(CheckIPOSupported)

# Check if IPO is supported
check_ipo_supported(RESULT ipo_supported)
if(ipo_supported)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
  # Set the number of parallel LTO jobs
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto=4")
endif()

# Build with release optimizations and debug symbols by default
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebug)
endif()

set(PYBIND11_PYTHON_VERSION 3)
set(PYBIND11_FINDPYTHON ON)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(bitbots_splines REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rot_conv REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(ros2_python_extension REQUIRED)

generate_parameter_library(bitbots_quintic_walk_parameters src/parameters.yaml)

rosidl_generate_interfaces(
  ${PROJECT_NAME} "msg/WalkDebug.msg" "msg/WalkEngineDebug.msg" DEPENDENCIES
  std_msgs geometry_msgs)

include_directories(include ${PYTHON_INCLUDE_DIRS})

add_compile_options(-Wall -Werror -Wno-unused -Wextra -Wpedantic)

set(SOURCES src/walk_visualizer.cpp src/walk_engine.cpp src/walk_stabilizer.cpp
            src/walk_ik.cpp src/walk_node.cpp)

add_executable(WalkNode ${SOURCES})

ament_target_dependencies(
  WalkNode
  ament_cmake
  backward_ros
  biped_interfaces
  bitbots_msgs
  bitbots_splines
  control_toolbox
  geometry_msgs
  moveit_ros_planning_interface
  nav_msgs
  rclcpp
  rot_conv
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")

target_link_libraries(WalkNode "${cpp_typesupport_target}"
                      bitbots_quintic_walk_parameters)

# create the lib
pybind11_add_module(libpy_quintic_walk SHARED src/walk_pywrapper.cpp ${SOURCES})
ament_target_dependencies(
  libpy_quintic_walk
  PUBLIC
  ament_cmake
  biped_interfaces
  bitbots_msgs
  bitbots_splines
  control_toolbox
  geometry_msgs
  moveit_ros_planning_interface
  nav_msgs
  rclcpp
  ros2_python_extension
  rot_conv
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros)

target_link_libraries(
  libpy_quintic_walk PRIVATE "${cpp_typesupport_target}"
                             bitbots_quintic_walk_parameters)

ament_python_install_package(bitbots_quintic_walk_py)

install(TARGETS libpy_quintic_walk
        DESTINATION "${PYTHON_INSTALL_DIR}/bitbots_quintic_walk_py")

enable_bitbots_docs()

install(TARGETS WalkNode DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
