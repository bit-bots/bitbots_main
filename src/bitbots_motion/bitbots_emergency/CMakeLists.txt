cmake_minimum_required(VERSION 3.12)
project(bitbots_emergency)

find_package(ament_cmake REQUIRED)
find_package(livelybot_msg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(SOURCE_PUBLISHER src/emergency_node_publisher.cpp)
set(SOURCE_LISTENER src/emergency_node_listener.cpp)

add_executable(EMERGENCY_NODE_PUBLISHER ${SOURCE_PUBLISHER})
add_executable(EMERGENCY_NODE_LISTENER ${SOURCE_LISTENER})

ament_target_dependencies(EMERGENCY_NODE_PUBLISHER rclcpp std_msgs)
ament_target_dependencies(EMERGENCY_NODE_LISTENER rclcpp std_msgs livelybot_msg)

install(TARGETS EMERGENCY_NODE_PUBLISHER DESTINATION lib/${PROJECT_NAME})
install(TARGETS EMERGENCY_NODE_LISTENER DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/publisher.sh DESTINATION share/${PROJECT_NAME}/scripts)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(livelybot_msg)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
