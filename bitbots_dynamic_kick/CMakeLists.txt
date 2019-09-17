cmake_minimum_required(VERSION 2.8.3)
project(bitbots_dynamic_kick)
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        bio_ik
        bitbots_msgs
        bitbots_splines
        dynamic_reconfigure
        actionlib
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        visualization_msgs
        nav_msgs
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
        src/kick_node.cpp
        src/kick_engine.cpp
        src/stabilizer.cpp
        src/visualizer.cpp
        src/kick_ik.cpp)

add_executable(KickNode ${SOURCES})

add_dependencies(KickNode ${PROJECT_NAME}_gencfg)

target_link_libraries(KickNode ${catkin_LIBRARIES})
