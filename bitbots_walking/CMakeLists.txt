cmake_minimum_required(VERSION 2.8.3)
project(bitbots_walking)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  rospy
  std_msgs
  humanoid_league_msgs
  bitbots_common
)

# lib armadillo is needed for the used kinematic
find_package(Armadillo REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread date_time iostreams)

catkin_python_setup()


catkin_package(
  DEPENDS Eigen
)

add_subdirectory(src/bitbots_walking)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
