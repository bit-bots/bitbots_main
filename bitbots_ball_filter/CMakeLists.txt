cmake_minimum_required(VERSION 3.0.2)
project(bitbots_ball_filter)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  tf2_geometry_msgs
  humanoid_league_msgs
  dynamic_reconfigure
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
 )

catkin_python_setup()

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/BallFilter.cfg
  #...
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bitbots_ball_filter
#  CATKIN_DEPENDS geometry_msgs rospy
#  DEPENDS system_lib
)
