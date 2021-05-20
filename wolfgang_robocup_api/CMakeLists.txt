cmake_minimum_required(VERSION 2.8.3)
project(wolfgang_robocup_api)

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

protobuf_generate_python(PY_SOURCES proto/messages.proto)
#protobuf_generate_python(PROTO_PY proto/messages.proto)
add_custom_target(myTarget ALL
  DEPENDS ${PY_SOURCES})
