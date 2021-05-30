cmake_minimum_required(VERSION 2.8.3)
project(humanoid_league_team_communication)
add_compile_options(-Wall -Werror -Wno-unused)

include(FindProtobuf)
find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        humanoid_league_msgs
        geometry_msgs
        bitbots_docs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}_lib
)

include_directories(
        include
        ${Protobuf_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR} # Dir is necessary for the generated protobuf header files
)

protobuf_generate_cpp(PROTO_SRC PROTO_HEADER src/humanoid_league_team_communication/RobocupProtocol/robocup_extension.proto)
add_library(robocup_protocol_lib ${PROTO_HEADER} ${PROTO_SRC})

add_executable(team_communication
        src/${PROJECT_NAME}/team_communication.cpp
        include/${PROJECT_NAME}/team_communication.h
        src/${PROJECT_NAME}/udp_connection.cpp
)
target_link_libraries(team_communication ${catkin_LIBRARIES} robocup_protocol_lib ${Protobuf_LIBRARIES})
add_dependencies(team_communication ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

enable_bitbots_docs()
