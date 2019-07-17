cmake_minimum_required(VERSION 2.8.3)
project(bitbots_quintic_walk)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(SOURCES
        src/Footstep.cpp
        src/WalkEngine.cpp
        src/QuinticWalkingNode.cpp
        src/TrajectoryUtils.cpp
        )

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosconsole
        std_msgs
        humanoid_league_msgs
        dynamic_reconfigure
        eigen_conversions
        tf
        tf_conversions
        bio_ik
        moveit_core
        moveit_ros_move_group
        bitbots_ik
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
        WalkingDebug.msg
)

generate_dynamic_reconfigure_options(
        cfg/bitbots_quintic_walk_params.cfg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS humanoid_league_msgs std_msgs geometry_msgs bitbots_ik bitbots_splines message_runtime bitbots_msgs
        DEPENDS EIGEN3
)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


set(CODE_LIBRARIES
        urdfdom_world
        urdfdom_model
        )

add_library(${PROJECT_NAME} ${SOURCES})


add_executable(QuinticWalkingNode ${SOURCES})

add_dependencies(QuinticWalkingNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CODE_LIBRARIES})

target_link_libraries(QuinticWalkingNode ${catkin_LIBRARIES} ${PROJECT_NAME})
