cmake_minimum_required(VERSION 2.8.3)
project(bitbots_ik)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  std_msgs
  geometry_msgs
  eigen_conversions
  tf
  tf_conversions
  bio_ik
  moveit_core
  moveit_ros_move_group
  moveit_ros_planning_interface
)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning
        roscpp
        tf
        tf_conversions
)


include_directories(
        include 
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")


set(SOURCES
    src/AnalyticIKSolver.cpp
    src/BioIKSolver.cpp
)

set(CODE_LIBRARIES )

add_library(${PROJECT_NAME} ${SOURCES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
