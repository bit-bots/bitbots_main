cmake_minimum_required(VERSION 3.5)
project(bitbots_localization)

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
find_package(bitbots_utils REQUIRED)
find_package(
  Boost
  COMPONENTS filesystem
  REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(particle_filter REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(soccer_vision_3d_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

generate_parameter_library(
  localization_parameters # cmake target name for the parameter library
  src/parameters.yml)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/ResetFilter.srv"
                           "srv/SetPaused.srv" DEPENDENCIES builtin_interfaces)

include_directories(${INCLUDE_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES})

# Compile as C++17
add_compile_options(-Wall -Werror -Wno-unused)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages

find_package(Eigen3 REQUIRED)

find_package(OpenCV)

# Generate services in the 'srv' folder

# Generate added messages and services with any dependencies listed here

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations

include_directories(include)

# Declare a C++ library
set(SOURCES
    src/localization.cpp
    src/map.cpp
    src/MotionModel.cpp
    src/ObservationModel.cpp
    src/RobotState.cpp
    src/StateDistribution.cpp
    src/tools.cpp)

# Add cmake target dependencies of the library as an example, code may need to
# be generated before libraries either from message generation or dynamic
# reconfigure#include "ObservationModel.h"

# Declare a C++ executable With catkin_make all packages are built within a
# single CMake context The recommended prefix ensures that target names across
# packages don't collide
add_executable(localization ${SOURCES})

ament_target_dependencies(
  localization
  ament_cmake
  ament_index_cpp
  bitbots_utils
  Boost
  cv_bridge
  image_transport
  nav_msgs
  OpenCV
  particle_filter
  rclcpp
  soccer_vision_3d_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")

target_link_libraries(localization "${cpp_typesupport_target}"
                      localization_parameters)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS localization DESTINATION lib/${PROJECT_NAME})

ament_package()
