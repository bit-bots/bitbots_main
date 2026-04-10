cmake_minimum_required(VERSION 3.0.2)
project(livelybot_bringup)

# add_definitions(-DDEBUG)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  serial  
  livelybot_msg  
  livelybot_serial
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(canboard_update src/canboard_update.cpp)
target_link_libraries(canboard_update ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(motor_feedback src/motor_feedback.cpp)
target_link_libraries(motor_feedback ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(motor_rerun src/motor_rerun.cpp)
target_link_libraries(motor_rerun ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(motor_move_zero src/motor_move_zero.cpp)
target_link_libraries(motor_move_zero ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(motor_set_zero src/motor_set_zero.cpp)
target_link_libraries(motor_set_zero ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(motor_timeout src/motor_timeout.cpp)
target_link_libraries(motor_timeout ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

install(TARGETS canboard_update
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS motor_feedback
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS motor_move_zero
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS motor_rerun
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS motor_set_zero
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS motor_timeout
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
 
install(DIRECTORY script/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/script
)
