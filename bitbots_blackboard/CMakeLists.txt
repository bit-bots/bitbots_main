cmake_minimum_required(VERSION 3.0.2)
project(bitbots_blackboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  bitbots_docs
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  bio_ik_msgs
  humanoid_league_msgs
  bitbots_msgs
)

catkin_python_setup()

catkin_package()

enable_bitbots_docs()
