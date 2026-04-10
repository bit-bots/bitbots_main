cmake_minimum_required(VERSION 3.0.2)
project(livelybot_oled)


message(${CMAKE_HOST_SYSTEM_PROCESSOR})

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  serial  
  livelybot_msg  
  livelybot_power
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES livelybot_oled
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  # DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(OLED_NODE_FILE
    src/sensor_actuator_status.cpp
    src/ip_addr.cpp
    src/oled_interface.cpp
)

find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

add_library(livelybot_oled STATIC ${OLED_NODE_FILE})
target_link_libraries(livelybot_oled ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

link_directories(lib)

add_executable(livelybot_oled_node src/oled_run.cpp)
target_link_libraries(livelybot_oled_node livelybot_oled ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(livelybot_oled_hd_test src/oled_hardware_test.cpp)
target_link_libraries(livelybot_oled_hd_test livelybot_oled ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

 install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)