cmake_minimum_required(VERSION 2.8.3)
project(bitbots_convenience_frames)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  std_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf_conversions
  message_generation
  geometry_msgs
  rospy
  humanoid_league_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(convenience_frames src/convenience_frames.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(convenience_frames
    ${catkin_LIBRARIES}
)
