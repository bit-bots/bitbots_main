cmake_minimum_required(VERSION 2.8.3)
project(bitbots_teleop)

find_package(catkin REQUIRED COMPONENTS
 message_generation
 humanoid_league_msgs
 rospy
 bitbots_docs
)

catkin_python_setup()

catkin_package()

enable_bitbots_docs()
