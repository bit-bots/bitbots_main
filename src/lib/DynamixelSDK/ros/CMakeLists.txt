################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(INCLUDE_DIRS include/dynamixel_sdk ${rclpy_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}  ${rclcpp_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${rclpy_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS}
  ${rclcpp_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${rclpy_LIBRARIES} ${ament_cmake_LIBRARIES} ${rclcpp_LIBRARIES})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -fPIC)

################################################################################
# Build
################################################################################

if(APPLE)
  add_library(dynamixel_sdk src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/group_bulk_read.cpp src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/port_handler.cpp src/dynamixel_sdk/port_handler_mac.cpp)
else()
  add_library(dynamixel_sdk src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/group_bulk_read.cpp src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/port_handler.cpp src/dynamixel_sdk/port_handler_linux.cpp)
endif()

target_link_libraries(dynamixel_sdk ${LIBS})

################################################################################
# Install
################################################################################
install(TARGETS dynamixel_sdk
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(PROGRAMS src/${PROJECT_NAME}/group_bulk_read.py
  src/${PROJECT_NAME}/group_bulk_write.py src/${PROJECT_NAME}/group_sync_read.py
  src/${PROJECT_NAME}/group_sync_write.py src/${PROJECT_NAME}/packet_handler.py
  src/${PROJECT_NAME}/port_handler.py
  src/${PROJECT_NAME}/protocol1_packet_handler.py
  src/${PROJECT_NAME}/protocol2_packet_handler.py
  src/${PROJECT_NAME}/robotis_def.py
  DESTINATION lib/${PROJECT_NAME})

################################################################################
# Test
################################################################################

ament_export_dependencies(rclpy)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_include_directories(include)
ament_export_libraries(dynamixel_sdk)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
