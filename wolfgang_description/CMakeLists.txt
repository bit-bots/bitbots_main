cmake_minimum_required(VERSION 2.8.3)
project(wolfgang_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  urdf
  xacro
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
