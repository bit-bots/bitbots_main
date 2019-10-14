cmake_minimum_required(VERSION 2.8.3)
project(humanoid_league_team_communication)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    humanoid_league_msgs
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_lib
)

include_directories(
    include
    src/humanoid_league_team_communication/mitecom
    ${catkin_INCLUDE_DIRS}
)


add_library(mitecom_lib
    src/${PROJECT_NAME}/mitecom/mitecom.cpp
    src/${PROJECT_NAME}/mitecom/mitecom.hpp
    src/${PROJECT_NAME}/mitecom/mitecom-data.h
    src/${PROJECT_NAME}/mitecom/mitecom-handler.cpp
    src/${PROJECT_NAME}/mitecom/mitecom-handler.h
    src/${PROJECT_NAME}/mitecom/mitecom-network.cpp
    src/${PROJECT_NAME}/mitecom/mitecom-network.h
)
add_dependencies(mitecom_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(mitecom_lib ${catkin_LIBRARIES})

add_executable(team_communication
    src/${PROJECT_NAME}/team_communication.cpp
    include/${PROJECT_NAME}/team_communication.h
)
target_link_libraries(team_communication ${catkin_LIBRARIES} mitecom_lib)
add_dependencies(team_communication ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
