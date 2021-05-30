cmake_minimum_required(VERSION 2.8.3)
project(bitbots_head_behavior)
add_compile_options(-Wall -Werror -Wno-unused)

find_package(Eigen3 REQUIRED)
find_package(PythonLibs)
if(PYTHONLIBS_VERSION_STRING VERSION_LESS "3.8")
    find_package(Boost REQUIRED COMPONENTS python3)
else()
    find_package(Boost REQUIRED COMPONENTS python)
endif()

find_package(PythonLibs COMPONENTS Interpreter Development)
find_package(catkin REQUIRED COMPONENTS std_msgs bitbots_docs moveit_core moveit_ros_robot_interaction moveit_ros_move_group)

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS EIGEN3
)

enable_bitbots_docs()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

# create the lib
add_library(collision_checker SHARED src/collision_checker.cpp)
add_dependencies(collision_checker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# link
target_link_libraries(collision_checker ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CODE_LIBRARIES})

set_target_properties(collision_checker PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
