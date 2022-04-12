cmake_minimum_required(VERSION 3.5)
project(bitbots_localization)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(bitbots_test REQUIRED)
find_package(ament_cmake REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/ResetFilter.srv"
  "srv/SetPaused.srv"
  DEPENDENCIES builtin_interfaces)

set(INCLUDE_DIRS include ${bitbots_test_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${bitbots_test_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${bitbots_test_LIBRARIES} ${ament_cmake_LIBRARIES})

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Werror -Wno-unused)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Eigen3 REQUIRED)

find_package(OpenCV)

## Generate services in the 'srv' folder

## Generate added messages and services with any dependencies listed here

## Generate dynamic reconfigure parameters in the 'cfg' folder


include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

## Declare a C++ library
add_library(${PROJECT_NAME}_lib include/bitbots_localization/map.h
  include/bitbots_localization/MotionModel.h
  include/bitbots_localization/ObservationModel.h
  include/bitbots_localization/RobotState.h
  include/bitbots_localization/StateDistribution.h
  include/bitbots_localization/tools.h src/map.cpp src/MotionModel.cpp
  src/ObservationModel.cpp src/RobotState.cpp src/StateDistribution.cpp
  src/tools.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure#include "ObservationModel.h"

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(localization src/localization.cpp)

## Add cmake target dependencies of the executable
## same as for the library above

## Specify libraries to link a library or executable target against
target_link_libraries(localization ${OpenCV_LIBRARIES} ${PROJECT_NAME}_lib
  ${LIBS})

## Testing
if(BUILD_TESTING)
  enable_bitbots_tests()
endif()

ament_export_dependencies(bitbots_test)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME}_lib ${LIBS})

install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
