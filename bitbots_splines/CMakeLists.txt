cmake_minimum_required(VERSION 2.8.3)
project(bitbots_splines)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosconsole
        std_msgs
        eigen_conversions
        bio_ik
        )
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

catkin_python_setup()


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        DEPENDS EIGEN3
)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")


set(SOURCES
		src/Spline/polynom.cpp
        src/Spline/smooth_spline.cpp
		src/Spline/spline.cpp
        src/Spline/pose_spline.cpp
        src/Spline/position_spline.cpp
		src/Utils/newton_binomial.cpp
		src/Utils/combination.cpp
        include/bitbots_splines/abstract_engine.h
        include/bitbots_splines/abstract_stabilizer.h
        include/bitbots_splines/abstract_ik.h
        include/bitbots_splines/abstract_visualizer.h
	include/bitbots_splines/reference_goals.h
	include/bitbots_splines/dynamic_balancing_goal.h
        )

set(CODE_LIBRARIES)

add_library(${PROJECT_NAME} ${SOURCES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
