cmake_minimum_required(VERSION 3.5)
project(bitbots_odometry)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(rotconv REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_compile_options(-Wall -Werror -Wno-unused)

add_executable(odometry_fuser src/odometry_fuser.cpp)
add_executable(motion_odometry src/motion_odometry.cpp)

## Specify libraries to link a library or executable target against
ament_target_dependencies(motion_odometry
    tf2_eigen
    tf2
    nav_msgs
    message_filters
    tf2_geometry_msgs
    bitbots_msgs
    geometry_msgs
    bitbots_docs
    rotconv
    sensor_msgs
    tf2_ros
    ament_cmake
    rclcpp)

ament_target_dependencies(odometry_fuser
    tf2_eigen
    tf2
    nav_msgs
    message_filters
    tf2_geometry_msgs
    bitbots_msgs
    geometry_msgs
    bitbots_docs
    rotconv
    sensor_msgs
    tf2_ros
    ament_cmake
    rclcpp)
target_link_libraries(odometry_fuser ${Eigen3_LIBRARIES})

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

ament_export_dependencies(tf2_eigen)
ament_export_dependencies(tf2)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(message_filters)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(bitbots_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(rotconv)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_include_directories(include)

ament_package()
