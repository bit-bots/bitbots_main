cmake_minimum_required(VERSION 2.8.3)
project(bitbots_dynup)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Werror -Wno-unused)

set(SOURCES
        src/dynup_node.cpp
        src/dynup_engine.cpp
        src/dynup_stabilizer.cpp
        src/dynup_ik.cpp
        src/visualizer.cpp
        src/dynup_pywrapper.cpp
        )

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(PythonLibs)
if(PYTHONLIBS_VERSION_STRING VERSION_LESS "3.8")
    find_package(Boost REQUIRED COMPONENTS python3)
else()
    find_package(Boost REQUIRED COMPONENTS python)
endif()
find_package (PythonLibs COMPONENTS Interpreter Development)


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

find_package(PkgConfig REQUIRED)

catkin_python_setup()
enable_bitbots_docs()

generate_dynamic_reconfigure_options(
    cfg/bitbots_dynup_params.cfg
)

add_message_files(
        FILES
        DynupEngineDebug.msg
        DynupPoses.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS rotconv
)

add_executable(DynupNode ${SOURCES})

add_dependencies(DynupNode ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(DynupNode ${catkin_LIBRARIES} ${CODE_LIBRARIES})

# create the lib
add_library(py_dynup SHARED src/dynup_pywrapper.cpp ${SOURCES})
add_dependencies(py_dynup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# link
target_link_libraries(py_dynup ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CODE_LIBRARIES})

set_target_properties(py_dynup PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )

if (CATKIN_ENABLE_TESTING)
    find_package(catkin REQUIRED COMPONENTS bitbots_test)
    enable_bitbots_tests()
endif()
