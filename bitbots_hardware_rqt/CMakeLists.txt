cmake_minimum_required(VERSION 2.8.3)
project(hardware_info)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

catkin_python_setup()
## Generate messages in the 'msg' folder
add_message_files(
    FILES
  CpuMessage.msg
  BatteryMessage.msg
)



## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
 )

catkin_package(
   CATKIN_DEPENDS
   message_runtime
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)

