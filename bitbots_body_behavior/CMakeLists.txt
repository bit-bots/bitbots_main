cmake_minimum_required(VERSION 2.8.3)
project(bitbots_body_behavior)

find_package(catkin REQUIRED COMPONENTS rospy bitbots_docs)

catkin_python_setup()

catkin_package()

enable_bitbots_docs()

if (CATKIN_ENABLE_TESTING)
    find_package(catkin REQUIRED COMPONENTS bitbots_test)
    enable_bitbots_tests()
endif()
