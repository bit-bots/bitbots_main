cmake_minimum_required(VERSION 2.8.3)
project(bitbots_blackboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  bitbots_docs
  sensor_msgs
  sensor_srvs
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
