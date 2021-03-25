cmake_minimum_required(VERSION 2.8.3)
project(bitbots_convenience_frames)

find_package(catkin REQUIRED COMPONENTS
  bitbots_docs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
  rospy
  humanoid_league_msgs
  bitbots_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ executable
add_executable(convenience_frames src/convenience_frames.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(convenience_frames
    ${catkin_LIBRARIES}
)

enable_bitbots_docs()
