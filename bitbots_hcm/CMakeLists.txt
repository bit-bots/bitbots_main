cmake_minimum_required(VERSION 2.8.3)
project(bitbots_hcm)

find_package(catkin REQUIRED COMPONENTS std_msgs humanoid_league_msgs rospy dynamic_reconfigure)

catkin_python_setup()

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/hcm_params.cfg
)

catkin_package(
 CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
