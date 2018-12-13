cmake_minimum_required(VERSION 2.8.3)
project(bitbots_hardware_rqt)

find_package(catkin)

catkin_package(CATKIN_DEPENDS)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)

