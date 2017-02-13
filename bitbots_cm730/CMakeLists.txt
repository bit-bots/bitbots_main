cmake_minimum_required(VERSION 2.8.3)

project(bitbots_cm730)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  roscpp
  std_msgs
  humanoid_league_msgs
  bitbots_common
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread date_time iostreams)

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SwitchMotorPower.srv
   SetLEDs.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
  LIBRARIES bitbots_cm730
  CATKIN_DEPENDS message_runtime
  DEPENDS Eigen
)

add_subdirectory(src/bitbots_cm730)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
