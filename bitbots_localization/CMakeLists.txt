cmake_minimum_required(VERSION 2.8.3)
project(bitbots_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  humanoid_league_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  robot_localization
  pointcloud_to_laserscan
  map_server
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
