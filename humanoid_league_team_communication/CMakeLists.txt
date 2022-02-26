cmake_minimum_required(VERSION 3.5)
project(humanoid_league_team_communication)

find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Protobuf REQUIRED)

protobuf_generate_python(PROTO_PY
  src/humanoid_league_team_communication/RobocupProtocol/robocup_extension.proto)
add_custom_target(humanoid_league_team_communication ALL DEPENDS ${PROTO_PY})
add_custom_command(TARGET humanoid_league_team_communication
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_PY}
  ${CMAKE_SOURCE_DIR}/src/humanoid_league_team_communication)

enable_bitbots_docs()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_package()
