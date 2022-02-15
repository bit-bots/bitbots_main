cmake_minimum_required(VERSION 3.5)
project(bitbots_moveit_bindings)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(pybind11 REQUIRED)

add_compile_options(-Wall -Werror -Wno-unused)

find_package(PythonLibs COMPONENTS Interpreter Development)
if(PYTHONLIBS_VERSION_STRING VERSION_LESS "3.8")
  find_package(Boost REQUIRED COMPONENTS python3)
else()
  find_package(Boost REQUIRED COMPONENTS python)
endif()

#include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
#enable_bitbots_docs()

add_library(bitbots_moveit_bindings SHARED src/bitbots_moveit_bindings.cpp)
target_include_directories(bitbots_moveit_bindings
  PRIVATE ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
target_link_libraries(bitbots_moveit_bindings ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} moveit_py_bindings_tools)

ament_target_dependencies(bitbots_moveit_bindings
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    moveit_ros_planning
    moveit_ros)

ament_export_dependencies(bitbots_docs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_ros_planning_interface)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(moveit_ros)

install(TARGETS bitbots_moveit_bindings DESTINATION ${PYTHON_INSTALL_DIR}/bitbots_moveit_bindings)
ament_python_install_package(${PROJECT_NAME})

#pybind11_add_module(mymodule src/bitbots_moveit_bindings.cpp)
#install(TARGETS bitbots_moveit_bindings
#  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
#)


ament_package()
