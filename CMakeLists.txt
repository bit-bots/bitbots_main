cmake_minimum_required(VERSION 2.8.3)
project(bitbots_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  FILES
  ColorSpace.msg
  Config.msg
  Cpu.msg
  FootPressure.msg
  JointCommand.msg
  JointTorque.msg
)

add_service_files(
   FILES
   ManualPenalize.srv
   FootScale.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)
