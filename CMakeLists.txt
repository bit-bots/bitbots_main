cmake_minimum_required(VERSION 3.1)
project(visual_compass)

# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  humanoid_league_msgs
  bitbots_msgs
  trajectory_msgs
  message_generation
  bitbots_vision
)

catkin_python_setup()

FIND_PACKAGE (OpenCV REQUIRED)

#add dynamic reconfigure api
find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/VisualCompass.cfg
  #...
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs humanoid_league_msgs image_transport message_runtime rospy rospkg sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")