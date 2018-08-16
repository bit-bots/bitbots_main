cmake_minimum_required(VERSION 2.8.3)
project(bitbots_common)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(BitBotsIncludes)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole rospy humanoid_league_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread date_time iostreams)
find_package(Eigen REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package(
  #LIBRARIES bitbots_common
  DEPENDS Eigen
  CFG_EXTRAS
    BitBotsIncludes.cmake
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


install(DIRECTORY cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


