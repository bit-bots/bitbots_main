cmake_minimum_required(VERSION 3.0.2)
project(livelybot_power)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  Power_switch.msg
  Power_detect.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES livelybot_power
 CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(livelybot_can_power STATIC src/livelybot_can_driver.cpp src/livelybot_power.cpp)
add_dependencies(livelybot_can_power ${catkin_EXPORTED_TARGETS} livelybot_power_gencpp)
target_link_libraries(livelybot_can_power pthread ${catkin_LIBRARIES})

add_executable(power_node src/power_class.cpp)
add_dependencies(power_node ${catkin_EXPORTED_TARGETS} livelybot_power_gencpp)
target_link_libraries(power_node livelybot_can_power pthread ${catkin_LIBRARIES} )

install(TARGETS power_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS livelybot_can_power
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
