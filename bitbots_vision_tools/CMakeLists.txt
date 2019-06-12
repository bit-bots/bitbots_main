cmake_minimum_required(VERSION 2.8.3)
project(bitbots_vision_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  humanoid_league_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  message_generation
  bitbots_vision
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
	cfg/ColorTest.cfg
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs humanoid_league_msgs image_transport message_runtime rospy sensor_msgs std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable( colorspace_tool src/colorspace_tool.cpp )

target_link_libraries( colorspace_tool ${catkin_LIBRARIES} yaml-cpp )

