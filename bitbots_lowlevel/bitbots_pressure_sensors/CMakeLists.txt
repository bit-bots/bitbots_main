cmake_minimum_required(VERSION 3.5)
project(bitbots_pressure_sensors)

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
find_package(bitbots_msgs REQUIRED)
# find_package(controller_interface REQUIRED) find_package(controller_manager
# REQUIRED)
find_package(diagnostic_msgs REQUIRED)
# find_package(hardware_interface REQUIRED) find_package(moveit_core REQUIRED)
# find_package(moveit_ros_planning REQUIRED) find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(transmission_interface REQUIRED)
find_package(yaml-cpp REQUIRED)

set(INCLUDE_DIRS include)
include_directories(${INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Werror -Wno-unused -pedantic -Wextra)

set(SOURCES src/pressure_converter.cpp)

# enable_bitbots_docs()

add_executable(pressure_converter ${SOURCES})
ament_target_dependencies(
  pressure_converter
  ament_cmake
  ament_index_cpp
  bitbots_msgs
  # controller_interface controller_manager
  diagnostic_msgs
  # hardware_interface moveit_core moveit_ros_planning pluginlib
  rclcpp
  rosidl_typesupport_cpp
  std_msgs
  std_srvs
  tf2_ros
  # transmission_interface
  yaml-cpp)
target_link_libraries(pressure_converter yaml-cpp)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(bitbots_msgs)
# ament_export_dependencies(controller_interface)
# ament_export_dependencies(controller_manager)
# ament_export_dependencies(hardware_interface)
# ament_export_dependencies(pluginlib)
ament_export_dependencies(std_msgs)
ament_export_dependencies(tf2_ros)
# ament_export_dependencies(transmission_interface)

install(TARGETS pressure_converter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts/ USE_SOURCE_PERMISSIONS
        DESTINATION lib/${PROJECT_NAME})

ament_package()
