cmake_minimum_required(VERSION 2.8.3)
project(bitbots_quintic_walk)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(SOURCES
        src/walk_visualizer.cpp
        src/walk_engine.cpp
        src/walk_stabilizer.cpp
        src/walk_ik.cpp
        src/walk_node.cpp
        )

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosconsole
        std_msgs
        humanoid_league_msgs
        dynamic_reconfigure
        tf2
        tf2_eigen
        tf2_geometry_msgs
        bio_ik
        moveit_core
        moveit_ros_move_group
        bitbots_splines
        message_generation
        message_runtime
        geometry_msgs
        moveit_ros_robot_interaction
        bitbots_msgs
        )

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

catkin_python_setup()

add_message_files(
        FILES
        WalkDebug.msg
        WalkEngineDebug.msg
)

generate_dynamic_reconfigure_options(
        cfg/bitbots_quintic_walk_params.cfg
        cfg/bitbots_quintic_walk_engine_params.cfg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS humanoid_league_msgs std_msgs geometry_msgs bitbots_splines message_runtime bitbots_msgs tf2 tf2_eigen tf2_geometry_msgs
        DEPENDS EIGEN3
)



set(CODE_LIBRARIES
        urdfdom_world
        urdfdom_model
)

add_executable(WalkNode ${SOURCES})

add_dependencies(WalkNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(WalkNode ${catkin_LIBRARIES} ${CODE_LIBRARIES})
