cmake_minimum_required(VERSION 3.1)
project(bitbots_visual_compass)

# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  humanoid_league_msgs
  geometry_msgs
  message_generation
  dynamic_reconfigure
)

catkin_python_setup()

FIND_PACKAGE (OpenCV REQUIRED)

#add dynamic reconfigure api
find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/visualCompass.cfg
  #...
)

catkin_package(
  CATKIN_DEPENDS humanoid_league_msgs rospy rospkg std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")