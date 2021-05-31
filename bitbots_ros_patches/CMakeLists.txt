cmake_minimum_required(VERSION 2.8.3)
project(bitbots_ros_patches)

find_package(catkin REQUIRED COMPONENTS
 rospy
 bitbots_docs
)

catkin_python_setup()

catkin_package()

enable_bitbots_docs()
