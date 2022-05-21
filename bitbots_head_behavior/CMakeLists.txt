cmake_minimum_required(VERSION 3.5)
project(bitbots_head_behavior)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_robot_interaction REQUIRED)
find_package(pybind11 REQUIRED)
find_package(ros2_python_extension REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(INCLUDE_DIRS include)
include_directories(${INCLUDE_DIRS})
add_compile_options(-Wall -Werror -Wno-unused)

find_package(Eigen3 REQUIRED)
find_package(PythonLibs)
find_package(PythonLibs COMPONENTS Interpreter Development)

enable_bitbots_docs()

# create the lib
pybind11_add_module(collision_checker SHARED src/collision_checker.cpp)
ament_target_dependencies(collision_checker PUBLIC
      ament_cmake
      moveit_core
      moveit_ros
      rclcpp
      ros2_python_extension)

install(TARGETS collision_checker
        DESTINATION "${PYTHON_INSTALL_DIR}/collision_checker_py"
        )


ament_export_dependencies(ament_cmake)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_ros_move_group)
ament_export_dependencies(moveit_ros_robot_interaction)
ament_export_dependencies(std_msgs)

ament_package()
