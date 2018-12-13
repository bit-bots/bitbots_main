cmake_minimum_required(VERSION 2.8.3)
project(bitbots_animation_server)


find_package(catkin REQUIRED COMPONENTS rospy humanoid_league_msgs message_generation actionlib trajectory_msgs genmsg std_msgs)

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS bitbots_common
)
