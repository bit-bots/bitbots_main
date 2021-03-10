cmake_minimum_required(VERSION 3.0)
project(bitbots_test VERSION 0.0.1)

find_package(catkin REQUIRED COMPONENTS bitbots_docs)

catkin_python_setup()

enable_bitbots_docs()

catkin_package(CFG_EXTRAS enable_bitbots_tests.cmake)
