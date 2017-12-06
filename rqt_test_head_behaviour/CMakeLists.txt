cmake_minimum_required(VERSION 2.8.3)
project(rqt_test_head_behaviour)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rqt_test_head_behaviour
#  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

