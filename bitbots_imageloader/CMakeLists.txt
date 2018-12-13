cmake_minimum_required(VERSION 2.8.3)
project(bitbots_imageloader)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  humanoid_league_msgs
  geometry_msgs
  nav_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)