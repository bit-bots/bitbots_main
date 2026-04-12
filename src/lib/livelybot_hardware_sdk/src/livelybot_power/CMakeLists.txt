cmake_minimum_required(VERSION 3.8)
project(livelybot_power)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livelybot_msg REQUIRED)

# Pure SocketCAN driver library (no ROS dependency)
add_library(livelybot_can_driver STATIC src/livelybot_can_driver.cpp)
target_include_directories(livelybot_can_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(livelybot_can_driver pthread)

# Power board ROS 2 node
add_executable(power_node src/power_node.cpp)
ament_target_dependencies(power_node rclcpp std_msgs livelybot_msg)
target_link_libraries(power_node livelybot_can_driver "${cpp_typesupport_target}")
target_include_directories(power_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS power_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
