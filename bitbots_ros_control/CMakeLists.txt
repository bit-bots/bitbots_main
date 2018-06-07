cmake_minimum_required(VERSION 2.8.3)
project(bitbots_ros_control)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  dynamixel_workbench_toolbox
  hardware_interface
  controller_manager
  dynamic_reconfigure
  humanoid_league_msgs
  pluginlib
  controller_interface
)

add_message_files(
  FILES
  JointCommand.msg
  JointTorque.msg
)

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

set(HEADERS
  include/${PROJECT_NAME}/dynamixel_hardware_interface.h
  include/${PROJECT_NAME}/dynamixel_controller.hpp
  include/${PROJECT_NAME}/posvelacccur_command_interface.h
)

set(SOURCES
  src/dynamixel_hardware_interface.cpp
  src/dynamixel_controller.cpp
)

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
  cfg/cpp
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
