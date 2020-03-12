cmake_minimum_required(VERSION 2.8.3)
project(bitbots_ros_control)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_workbench_toolbox
  hardware_interface
  controller_manager
  dynamic_reconfigure
  humanoid_league_msgs
  pluginlib
  controller_interface
  bitbots_buttons
  transmission_interface
  realtime_tools
  bitbots_msgs
)
find_package(yaml-cpp REQUIRED)

set(SOURCES
  src/dynamixel_servo_hardware_interface.cpp
  src/imu_hardware_interface.cpp
  src/bitfoot_hardware_interface.cpp
  src/button_hardware_interface.cpp
  src/dynamixel_controller.cpp
  src/wolfgang_hardware_interface.cpp
  src/node.cpp
  src/utils.cpp
)

generate_dynamic_reconfigure_options(
        cfg/dynamixel_servo_hardware_interface_params.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs dynamixel_workbench_toolbox hardware_interface controller_manager controller_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(node ${SOURCES})
add_dependencies(node ${PROJECT_NAME}_gencfg)
target_link_libraries(node ${catkin_LIBRARIES})

add_executable(pressure_converter src/pressure_converter.cpp)
target_link_libraries(pressure_converter ${catkin_LIBRARIES} yaml-cpp)
