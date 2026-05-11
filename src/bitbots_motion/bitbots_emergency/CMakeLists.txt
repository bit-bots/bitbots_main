cmake_minimum_required(VERSION 3.12)
project(bitbots_emergency)

find_package(ament_cmake REQUIRED)
find_package(livelybot_msg REQUIRED)
find_package(rclcpp REQUIRED)

set(SOURCES src/emergency_node.cpp)

add_executable(EMERGENCY_BUTTON ${SOURCES})

ament_target_dependencies(EMERGENCY_BUTTON rclcpp livelybot_msg)

install(TARGETS EMERGENCY_BUTTON DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(livelybot_msg)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
