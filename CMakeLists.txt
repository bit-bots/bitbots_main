cmake_minimum_required(VERSION 2.8.3)
project(bitbots_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_message_files(
  FILES
  JointCommand.msg
  JointTorque.msg
  FootPressure.msg
  CpuMessage.msg
  ImageWithCorners.msg
  PixelRelative.msg
  PixelsRelative.msg
  Colorspace.msg
  Config.msg
)

add_service_files(
   FILES
   ManualPenalize.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs
)