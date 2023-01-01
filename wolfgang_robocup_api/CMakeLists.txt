cmake_minimum_required(VERSION 3.5)
project(wolfgang_robocup_api)

find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED COMPONENTS

)

catkin_python_setup()

catkin_package()

protobuf_generate_python(PROTO_PY proto/messages.proto)
add_custom_target(robocup_api_proto ALL DEPENDS ${PROTO_PY})
add_custom_command(
  TARGET robocup_api_proto POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_PY} ${CMAKE_SOURCE_DIR}/src/wolfgang_robocup_api
)
