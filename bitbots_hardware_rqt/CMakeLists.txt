cmake_minimum_required(VERSION 2.8.3)
project(bitbots_hardware_rqt)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
)


add_message_files(
   FILES
   BatteryMessage.msg
   CpuMessage.msg
   HardwareMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

