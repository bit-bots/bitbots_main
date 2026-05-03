cmake_minimum_required(VERSION 3.12)
project(bitbots_emergency)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(SOURCES src/emergency_node.cpp)

add_executable(EMERGENCY_BUTTON ${SOURCES})

ament_target_dependencies(EMERGENCY_BUTTON rclcpp std_msgs)

install(TARGETS EMERGENCY_BUTTON DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
