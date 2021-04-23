cmake_minimum_required(VERSION 3.0.2)
project(bitbots_moveit_bindings)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning
    bitbots_docs
)

find_package(PythonLibs COMPONENTS Interpreter Development)
if(PYTHONLIBS_VERSION_STRING VERSION_LESS "3.8")
    find_package(Boost REQUIRED COMPONENTS python3)
else()
    find_package(Boost REQUIRED COMPONENTS python)
endif()

catkin_package(CATKIN_DEPENDS
  moveit_core
)

catkin_python_setup()

enable_bitbots_docs()

add_library(bitbots_moveit_bindings SHARED src/bitbots_moveit_bindings.cpp)
target_include_directories(bitbots_moveit_bindings PRIVATE ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
add_dependencies(bitbots_moveit_bindings ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bitbots_moveit_bindings ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(bitbots_moveit_bindings PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
