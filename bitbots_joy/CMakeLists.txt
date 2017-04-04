cmake_minimum_required(VERSION 2.8.3)
project(bitbots_joy)

find_package(catkin REQUIRED message_generation humanoid_league_msgs rospy joy)
catkin_python_setup()


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package()
