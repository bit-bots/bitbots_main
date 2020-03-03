cmake_minimum_required(VERSION 3.1)
project(bitbots_visual_compass)

# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  humanoid_league_msgs
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
  CATKIN_DEPENDS
  rospy
  rospkg
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  dynamic_reconfigure
  humanoid_league_msgs
  wolves_image_provider
  python3-cv_bridge
  python3-pyopencl
  python3-numpy
  python3-silx
  python3-opencv
  python3-yaml
  python3-matplotlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")