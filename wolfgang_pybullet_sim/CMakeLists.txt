cmake_minimum_required(VERSION 2.8.3)
project(wolfgang_pybullet_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  urdf
  bitbots_msgs
  nav_msgs
  rosgraph_msgs
  std_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
