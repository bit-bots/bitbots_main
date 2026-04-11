cmake_minimum_required(VERSION 3.8)
project(livelybot_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livelybot_msg REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(serial REQUIRED)

set(serial_SRCS
  src/lively_serial.cc
  src/hardware/motor.cc
  src/hardware/canport.cc
  src/hardware/canboard.cc
  src/hardware/robot.cc
  src/crc/crc8.cc
  src/crc/crc16.cc
)

add_library(livelybot_serial SHARED ${serial_SRCS})
target_include_directories(livelybot_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/hardware>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/crc>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(livelybot_serial
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  livelybot_msg
  bitbots_msgs
  diagnostic_updater
  serial
)
target_link_libraries(livelybot_serial
  rt
  pthread
)

add_executable(robot_node src/robot_node.cpp)
ament_target_dependencies(robot_node rclcpp sensor_msgs livelybot_msg)
target_link_libraries(robot_node livelybot_serial)

install(TARGETS livelybot_serial
  EXPORT livelybot_serialTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS robot_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_crc test/test_crc.cpp)
  target_link_libraries(test_crc livelybot_serial)

  ament_add_gtest(test_motor_conversion test/test_motor_conversion.cpp)
  # test_motor_conversion is fully self-contained (no ROS / hardware deps)
endif()

ament_export_targets(livelybot_serialTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(livelybot_serial)
ament_export_dependencies(rclcpp sensor_msgs livelybot_msg bitbots_msgs std_srvs diagnostic_updater serial)

ament_package()
