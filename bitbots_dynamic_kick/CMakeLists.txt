cmake_minimum_required(VERSION 2.8.3)
project(bitbots_dynamic_kick)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Werror -Wno-unused)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
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
        bitbots_docs
        control_toolbox
        message_generation
        message_runtime
        rotconv
        moveit_ros_planning
)

catkin_python_setup()

find_package(PythonLibs COMPONENTS Interpreter Development)
if(PYTHONLIBS_VERSION_STRING VERSION_LESS "3.8")
    find_package(Boost REQUIRED COMPONENTS python3)
else()
    find_package(Boost REQUIRED COMPONENTS python)
endif()

generate_dynamic_reconfigure_options(
        cfg/bitbots_dynamic_kick_params.cfg
)

add_message_files(
        FILES
        KickDebug.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
)

enable_bitbots_docs()

if (CATKIN_ENABLE_TESTING)
    find_package(catkin REQUIRED COMPONENTS bitbots_test)
    enable_bitbots_tests()
endif()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

set(SOURCES
        src/kick_node.cpp
        src/kick_engine.cpp
        src/stabilizer.cpp
        src/visualizer.cpp
        src/kick_ik.cpp)

add_executable(KickNode ${SOURCES})
add_dependencies(KickNode ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages)
target_link_libraries(KickNode ${catkin_LIBRARIES})

# create the kick python wrapper
add_library(py_dynamic_kick SHARED src/kick_pywrapper.cpp ${SOURCES})
add_dependencies(py_dynamic_kick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(py_dynamic_kick ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(py_dynamic_kick PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
