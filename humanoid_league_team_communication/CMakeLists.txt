cmake_minimum_required(VERSION 2.8.3)
project(humanoid_league_team_communication)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(Includes)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  humanoid_league_msgs
)

catkin_python_setup()

catkin_package()

add_subdirectory(src/humanoid_league_team_communication)

