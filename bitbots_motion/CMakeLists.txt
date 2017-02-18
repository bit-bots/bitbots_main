cmake_minimum_required(VERSION 2.8.3)
project(bitbots_motion)

find_package(catkin REQUIRED std_msgs humanoid_league_msgs rospy dynamic_reconfigure)

catkin_python_setup()

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/bitbots_motion_params.cfg
)

catkin_package(
 CATKIN_DEPENDS bitbots_common
)
