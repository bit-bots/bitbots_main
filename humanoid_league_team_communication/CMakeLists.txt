cmake_minimum_required(VERSION 2.8.3)
project(humanoid_league_team_communication)

find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        rospy
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        humanoid_league_msgs
        geometry_msgs
        bitbots_docs
)

catkin_python_setup()

catkin_package()

protobuf_generate_python(PROTO_PY src/humanoid_league_team_communication/RobocupProtocol/robocup_extension.proto)
add_custom_target(humanoid_league_team_communication ALL DEPENDS ${PROTO_PY})
add_custom_command(
  TARGET humanoid_league_team_communication POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_PY} ${CMAKE_SOURCE_DIR}/src/humanoid_league_team_communication
)


enable_bitbots_docs()
