cmake_minimum_required(VERSION 2.8.3)
project(bitbots_move_base)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    humanoid_league_msgs
    rospy
    move_base
    spatio_temporal_voxel_layer)


catkin_python_setup()

catkin_package(
)
