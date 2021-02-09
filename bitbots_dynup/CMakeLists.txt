cmake_minimum_required(VERSION 2.8.3)
project(bitbots_dynup)
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
    control_toolbox
    bitbots_docs
    rotconv
)

enable_bitbots_docs()

generate_dynamic_reconfigure_options(
    cfg/bitbots_dynup_params.cfg
)

add_message_files(
        FILES
        DynupEngineDebug.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS rotconv
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(SOURCES
    src/dynup_node.cpp
    src/dynup_engine.cpp
    src/dynup_stabilizer.cpp
    src/dynup_ik.cpp
    src/visualizer.cpp
)



add_executable(DynupNode ${SOURCES})

add_dependencies(DynupNode ${PROJECT_NAME}_gencfg)

target_link_libraries(DynupNode ${catkin_LIBRARIES})
