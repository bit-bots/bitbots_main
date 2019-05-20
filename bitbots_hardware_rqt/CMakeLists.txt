cmake_minimum_required(VERSION 2.8.3)
project(bitbots_hardware_rqt)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
  bitbots_msgs
)

catkin_python_setup()

add_message_files(
   FILES
   BatteryMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  bitbots_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime bitbots_msgs std_msgs)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

