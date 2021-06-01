cmake_minimum_required(VERSION 2.8.3)
project(bitbots_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  bitbots_docs
  roscpp
  rospy
  message_generation
  std_msgs
  sensor_msgs
)

add_action_files(
  DIRECTORY action
  FILES
  DynUp.action
  Kick.action
)

add_message_files(
  FILES
  ColorLookupTable.msg
  Config.msg
  Cpu.msg
  FootPressure.msg
  JointCommand.msg
  JointTorque.msg
  SupportState.msg
)

add_service_files(
   FILES
   IMURanges.srv
   ComplementaryFilterParams.srv
   ManualPenalize.srv
   FootScale.srv
   Leds.srv
   AccelerometerCalibration.srv
   SetAccelerometerCalibrationThreshold.srv
   SetObjectPose.srv
   SetObjectPosition.srv
)


generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs actionlib_msgs
)

enable_bitbots_docs()
