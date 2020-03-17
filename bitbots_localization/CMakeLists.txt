cmake_minimum_required(VERSION 2.8.3)
project(bitbots_localization)

## Compile as C++17
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  particle_filter
  roscpp
  roslib
  std_msgs
  visualization_msgs
  dynamic_reconfigure
  humanoid_league_msgs
  eigen_conversions
  tf2
  tf2_ros
  message_generation
        cv_bridge
        image_transport
)

find_package(Eigen3 REQUIRED)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

## Generate services in the 'srv' folder
add_service_files(
   FILES
   reset_filter.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/localization.cfg
)

catkin_package(
  INCLUDE_DIRS include
        LIBRARIES bitbots_localization_lib
        CATKIN_DEPENDS geometry_msgs particle_filter roscpp roslib std_msgs visualization_msgs humanoid_league_msgs dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
add_library(${PROJECT_NAME}_lib
        include/bitbots_localization/map.h
        include/bitbots_localization/MotionModel.h
        include/bitbots_localization/ObservationModel.h
        include/bitbots_localization/RobotState.h
        include/bitbots_localization/StateDistribution.h
        include/bitbots_localization/tools.h
        src/map.cpp
        src/MotionModel.cpp
        src/ObservationModel.cpp
        src/RobotState.cpp
        src/StateDistribution.cpp
        src/tools.cpp
        )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure#include "ObservationModel.h"
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(localization src/localization.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(localization
    ${OpenCV_LIBRARIES} ${PROJECT_NAME}_lib ${catkin_LIBRARIES}
)
