cmake_minimum_required(VERSION 2.8.3)
project(bitbots_dynamic_kick)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    bitbots_msgs
    dynamic_reconfigure
    actionlib
)

generate_dynamic_reconfigure_options(
    cfg/bitbots_dynamic_kick_params.cfg
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(SOURCES
    src/KickNode.cpp
    src/KickEngine.cpp
    src/Stabilizer.cpp
)

add_executable(KickNode ${SOURCES})

add_dependencies(KickNode ${PROJECT_NAME}_gencfg)

target_link_libraries(KickNode ${catkin_LIBRARIES})
