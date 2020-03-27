cmake_minimum_required(VERSION 2.8.3)
project(wolfgang_pybullet_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  urdf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
