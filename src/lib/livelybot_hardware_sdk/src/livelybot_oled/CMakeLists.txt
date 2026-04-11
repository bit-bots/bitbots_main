cmake_minimum_required(VERSION 3.8)
project(livelybot_oled)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)

# Static library for OLED hardware abstraction (no ROS dependency)
add_library(livelybot_oled_lib STATIC
  src/sensor_actuator_status.cpp
  src/ip_addr.cpp
  src/oled_interface.cpp
)
target_include_directories(livelybot_oled_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(livelybot_oled_lib rclcpp sensor_msgs std_msgs serial tf2)

# Main OLED node
add_executable(livelybot_oled_node src/oled_run.cpp)
target_link_libraries(livelybot_oled_node livelybot_oled_lib)
ament_target_dependencies(livelybot_oled_node rclcpp)

# Hardware test (standalone, no ROS)
add_executable(livelybot_oled_hd_test src/oled_hardware_test.cpp)
target_include_directories(livelybot_oled_hd_test PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(livelybot_oled_hd_test serial)
target_link_libraries(livelybot_oled_hd_test livelybot_oled_lib)

install(TARGETS livelybot_oled_node livelybot_oled_hd_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
