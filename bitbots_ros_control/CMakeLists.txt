cmake_minimum_required(VERSION 2.8.3)
project(bitbots_ros_control)

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

set(HEADERS
  include/${PROJECT_NAME}/dynamixel_hardware_interface.h
  include/${PROJECT_NAME}/imu_hardware_interface.h
  include/${PROJECT_NAME}/bitfoot_hardware_interface.h
  include/${PROJECT_NAME}/button_hardware_interface.h
  include/${PROJECT_NAME}/dynamixel_controller.hpp
  include/${PROJECT_NAME}/posvelacccur_command_interface.h
        src/wolfgang_hardware_interface.cpp include/bitbots_ros_control/wolfgang_hardware_interface.h)

set(SOURCES
        src/dynamixel_servo_hardware_interface.cpp
  src/imu_hardware_interface.cpp
  src/bitfoot_hardware_interface.cpp
  src/button_hardware_interface.cpp
  src/dynamixel_controller.cpp
        src/wolfgang_hardware_interface.cpp include/bitbots_ros_control/wolfgang_hardware_interface.h include/bitbots_ros_control/foot_pressure_sensor_interface.h include/bitbots_ros_control/bitfoot_controller.h src/bit_foot_controller.cpp)

generate_dynamic_reconfigure_options(
        cfg/bitbots_ros_control_params.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bitbots_ros_control
  CATKIN_DEPENDS roscpp std_msgs dynamixel_workbench_toolbox hardware_interface controller_manager controller_interface
)


add_compile_options(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(bitbots_ros_control
 ${HEADERS} ${SOURCES}
)
add_dependencies(bitbots_ros_control 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(bitbots_ros_control
  ${catkin_LIBRARIES}
)

add_executable(dynamixel_controller_manager_node src/dynamixel_controller_manager_node.cpp)

add_dependencies(dynamixel_controller_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(dynamixel_controller_manager_node bitbots_ros_control)
