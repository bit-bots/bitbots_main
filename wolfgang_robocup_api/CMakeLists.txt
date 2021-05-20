cmake_minimum_required(VERSION 2.8.3)
project(wolfgang_robocup_api)

include(FindProtobuf)
find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  bitbots_msgs
  humanoid_league_msgs
  rosgraph_msgs
)

catkin_python_setup()

catkin_package()

protobuf_generate_python(PROTO_PY proto/messages.proto)
add_custom_target(robocup_api_proto ALL DEPENDS ${PROTO_PY})
add_custom_command(
  TARGET robocup_api_proto POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_PY} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
