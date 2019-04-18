cmake_minimum_required(VERSION 2.8.3)
project(humanoid_league_game_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  humanoid_league_msgs
)

catkin_python_setup()

catkin_package()
