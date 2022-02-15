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
find_package (Python3 COMPONENTS Interpreter Development)

add_compile_options(-Wall -Werror -Wno-unused)

#include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
#enable_bitbots_docs()

add_library(bitbots_moveit_bindings src/bitbots_moveit_bindings.cpp)
target_link_libraries(bitbots_moveit_bindings pybind11:module)
target_include_directories(bitbots_moveit_bindings PRIVATE ${Python3_INCLUDE_DIRS})
pybind11_extension(bitbots_moveit_bindings)
pybind11_strip(bitbots_moveit_bindings)

set_target_properties(bitbots_moveit_bindings PROPERTIES
    CXX_VISIBILITY_PRESET "hidden"
    CUDA_VISIBILITY_PRESET "hidden")

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

ament_python_install_package(${PROJECT_NAME})

install(TARGETS bitbots_moveit_bindings
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_package()
