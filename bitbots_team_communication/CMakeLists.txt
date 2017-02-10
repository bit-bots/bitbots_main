cmake_minimum_required(VERSION 2.8.3)
project(bitbots_team_communication)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../bitbots_common/cmake) #  TODO: dynamic

include(BitBotsIncludes)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  humanoid_league_msgs
)

catkin_python_setup()

catkin_package()

add_subdirectory(src/bitbots_team_communication)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
