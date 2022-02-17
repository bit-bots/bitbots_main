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
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

add_compile_options(-Wall -Werror -Wno-unused)

#include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
#enable_bitbots_docs()

pybind11_add_module(libbitbots_moveit_bindings SHARED
  src/bitbots_moveit_bindings.cpp
)
ament_target_dependencies(libbitbots_moveit_bindings PUBLIC
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros
)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS libbitbots_moveit_bindings
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_package()
