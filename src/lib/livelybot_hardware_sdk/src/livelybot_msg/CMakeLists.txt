cmake_minimum_required(VERSION 3.0.2)
project(livelybot_msg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  test.msg
  MotorCmd.msg
  MotorState.msg
  HighState.msg
  IMU.msg
  LowCmd.msg
#   Message2.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs 
)

#############
## Install ##
#############
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)