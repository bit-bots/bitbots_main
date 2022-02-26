cmake_minimum_required(VERSION 3.5)
project(humanoid_league_team_communication)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)

set(INCLUDE_DIRS ${tf2_geometry_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS}
  ${humanoid_league_msgs_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS} ${bitbots_docs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS} ${rclpy_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${tf2_geometry_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS}
  ${humanoid_league_msgs_LIBRARY_DIRS} ${tf2_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS} ${bitbots_docs_LIBRARY_DIRS}
  ${std_msgs_LIBRARY_DIRS} ${rclpy_LIBRARY_DIRS} ${tf2_ros_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${tf2_geometry_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${humanoid_league_msgs_LIBRARIES} ${tf2_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${bitbots_docs_LIBRARIES} ${std_msgs_LIBRARIES} ${rclpy_LIBRARIES}
  ${tf2_ros_LIBRARIES})

find_package(Protobuf REQUIRED)

protobuf_generate_python(PROTO_PY
  src/humanoid_league_team_communication/RobocupProtocol/robocup_extension.proto)
add_custom_target(humanoid_league_team_communication ALL DEPENDS ${PROTO_PY})
add_custom_command(TARGET humanoid_league_team_communication
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_PY}
  ${CMAKE_SOURCE_DIR}/src/humanoid_league_team_communication)

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(humanoid_league_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rclpy)
ament_export_dependencies(tf2_ros)
ament_export_include_directories(${INCLUDE_DIRS})

install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
