cmake_minimum_required(VERSION 2.8.3)
project(bitbots_team_communication)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  humanoid_league_msgs
  bitbots_common
)

catkin_python_setup()

catkin_package()

add_subdirectory(src/bitbots_team_communication)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
