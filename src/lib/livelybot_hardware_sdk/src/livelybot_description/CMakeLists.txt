cmake_minimum_required(VERSION 3.0.2)
project(livelybot_description)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES livelybot_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY robot_param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robot_param
)

