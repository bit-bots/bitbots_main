cmake_minimum_required(VERSION 2.8.3)
project(bitbots_bringup)

find_package(catkin REQUIRED COMPONENTS bitbots_docs)

catkin_python_setup()

catkin_package()

include_directories()

enable_bitbots_docs()
