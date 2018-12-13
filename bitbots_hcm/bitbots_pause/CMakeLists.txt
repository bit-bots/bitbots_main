cmake_minimum_required(VERSION 2.8.3)
project(bitbots_pause)

find_package(catkin REQUIRED message_generation std_msgs humanoid_league_msgs rospy)

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
   FILES
   ManualPenalize.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package()
