cmake_minimum_required(VERSION 2.8.3)
project(bitbots_live_tool_rqt)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  bitbots_docs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS message_runtime)

enable_bitbots_docs()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
