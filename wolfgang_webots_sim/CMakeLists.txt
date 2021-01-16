cmake_minimum_required(VERSION 2.8.3)
project(wolfgang_webots_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  urdf
  bitbots_msgs
  nav_msgs
  rosgraph_msgs
  std_msgs
  bitbots_docs
)

catkin_python_setup()

enable_bitbots_docs()

catkin_package()

