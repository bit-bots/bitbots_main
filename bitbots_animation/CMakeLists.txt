cmake_minimum_required(VERSION 2.8.3)
project(bitbots_animation)


find_package(catkin REQUIRED COMPONENTS rospy humanoid_league_msgs message_generation actionlib trajectory_msgs genmsg std_msgs)

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
   FILES
   AnimationFrame.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
   PlayAnimation.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   trajectory_msgs
   actionlib_msgs
   sensor_msgs
 )

catkin_package(
  CATKIN_DEPENDS bitbots_common
)
