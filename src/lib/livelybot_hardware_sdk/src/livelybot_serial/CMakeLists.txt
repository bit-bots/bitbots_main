cmake_minimum_required(VERSION 3.0.2)
project(livelybot_serial)


# add_definitions(-DDEBUG)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    livelybot_msg  
    serial
    dynamic_reconfigure
)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(serial_SRCS
    src/lively_serial.cc
    src/hardware/motor.cc
    src/hardware/canport.cc
    src/hardware/canboard.cc
    src/hardware/robot.cc
    src/crc/crc8.cc
    src/crc/crc16.cc
)


generate_dynamic_reconfigure_options(
  cfg/robot_dynamic_config_20.cfg
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include include/crc include/hardware
    DEPENDS rt pthread
    CATKIN_DEPENDS roscpp std_msgs livelybot_msg serial dynamic_reconfigure
)
find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

include_directories(
  include
  include/crc
  include/hardware
  ${catkin_INCLUDE_DIRS}
)

add_library(livelybot_serial SHARED ${serial_SRCS})
add_dependencies(livelybot_serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(livelybot_serial ${catkin_LIBRARIES} rt pthread ${SERIALPORT_LIBRARY})

link_directories(lib)

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
# 安装头文件到 install/include
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
