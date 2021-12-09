cmake_minimum_required(VERSION 2.8.3)
project(bitbots_rl_motion)


find_package(catkin REQUIRED COMPONENTS rospy std_msgs bitbots_docs)
enable_bitbots_docs()

catkin_python_setup()


catkin_package()
