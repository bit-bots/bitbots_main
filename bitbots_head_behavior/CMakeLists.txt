cmake_minimum_required(VERSION 2.8.3)
project(bitbots_head_behavior)

find_package(catkin REQUIRED std_msgs humanoid_league_msgs rospy dynamic_stack_decider bitbots_ik)

catkin_python_setup()

catkin_package()
