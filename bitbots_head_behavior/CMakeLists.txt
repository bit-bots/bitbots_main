cmake_minimum_required(VERSION 2.8.3)
project(bitbots_head_behavior)

find_package(catkin REQUIRED COMPONENTS std_msgs bitbots_docs)

catkin_python_setup()

catkin_package()

enable_bitbots_docs()
