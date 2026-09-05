cmake_minimum_required(VERSION 3.8)
project(bitbots_head_mover)

# Build with -O2 optimizations and debug symbols by default, without disabling
# assertions (unlike CMake's built-in RelWithDebInfo, which defines NDEBUG).
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  add_compile_options(-O2 -g)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(bitbots_splines REQUIRED)
find_package(bitbots_utils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(soccer_vision_3d_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

generate_parameter_library(
  head_parameters # cmake target name for the parameter library
  config/head_config.yml)

# ---------------------------------------------------------------------------
# Head mover library – node independent pieces, so they can be unit tested
# ---------------------------------------------------------------------------
add_library(
  bitbots_head_mover_lib SHARED
  src/active_vision.cpp
  src/active_vision_debug.cpp
  src/active_vision_scorer.cpp
  src/camera_model.cpp
  src/field_coverage_map.cpp
  src/head_kinematics.cpp
  src/head_trajectory.cpp
  src/look_at.cpp
  src/search_pattern.cpp
  src/trajectory_sampler.cpp
  src/world_model.cpp)

target_include_directories(
  bitbots_head_mover_lib
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(bitbots_head_mover_lib PUBLIC Eigen3::Eigen
                                                    ${OpenCV_LIBS})

ament_target_dependencies(
  bitbots_head_mover_lib
  PUBLIC
  bitbots_splines
  geometry_msgs
  image_geometry
  kdl_parser
  nav_msgs
  orocos_kdl_vendor
  sensor_msgs
  urdf
  visualization_msgs)

# ---------------------------------------------------------------------------
# move_head node
# ---------------------------------------------------------------------------
add_executable(move_head src/move_head.cpp)
target_link_libraries(move_head rclcpp::rclcpp head_parameters
                      bitbots_head_mover_lib)

ament_target_dependencies(
  move_head
  backward_ros
  bitbots_msgs
  bitbots_splines
  bitbots_utils
  cv_bridge
  generate_parameter_library
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  soccer_vision_3d_msgs
  std_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_search_pattern test/test_search_pattern.cpp)
  target_link_libraries(test_search_pattern bitbots_head_mover_lib)

  ament_add_gtest(test_head_trajectory test/test_head_trajectory.cpp)
  target_link_libraries(test_head_trajectory bitbots_head_mover_lib)

  ament_add_gtest(test_look_at test/test_look_at.cpp)
  target_link_libraries(test_look_at bitbots_head_mover_lib)

  ament_add_gtest(test_types test/test_types.cpp)
  target_link_libraries(test_types bitbots_head_mover_lib)

  ament_add_gtest(test_head_kinematics test/test_head_kinematics.cpp)
  target_link_libraries(test_head_kinematics bitbots_head_mover_lib)

  ament_add_gtest(test_camera_model test/test_camera_model.cpp)
  target_link_libraries(test_camera_model bitbots_head_mover_lib)

  ament_add_gtest(test_world_model test/test_world_model.cpp)
  target_link_libraries(test_world_model bitbots_head_mover_lib)

  ament_add_gtest(test_field_coverage_map test/test_field_coverage_map.cpp)
  target_link_libraries(test_field_coverage_map bitbots_head_mover_lib)

  ament_add_gtest(test_trajectory_sampler test/test_trajectory_sampler.cpp)
  target_link_libraries(test_trajectory_sampler bitbots_head_mover_lib)

  ament_add_gtest(test_active_vision_scorer test/test_active_vision_scorer.cpp)
  target_link_libraries(test_active_vision_scorer bitbots_head_mover_lib)

  ament_add_gtest(test_active_vision test/test_active_vision.cpp)
  target_link_libraries(test_active_vision bitbots_head_mover_lib)
endif()

install(TARGETS move_head DESTINATION lib/${PROJECT_NAME})

install(TARGETS bitbots_head_mover_lib DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
