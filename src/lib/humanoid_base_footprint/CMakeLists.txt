cmake_minimum_required(VERSION 3.5)
project(humanoid_base_footprint)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(biped_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rot_conv REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  biped_interfaces
  Eigen3
  rclcpp
  rclcpp_components
  rot_conv
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros)

# Build
add_library(base_footprint_node SHARED
  src/base_footprint.cpp)

target_include_directories(base_footprint_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(base_footprint_node
  PLUGIN "humanoid_base_footprint::BaseFootprintBroadcaster"
  EXECUTABLE base_footprint)

ament_target_dependencies(base_footprint_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_base_footprint HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS base_footprint_node
  EXPORT export_base_footprint
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

## Run tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()
