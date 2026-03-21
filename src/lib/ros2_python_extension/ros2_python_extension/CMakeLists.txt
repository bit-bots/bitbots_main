cmake_minimum_required(VERSION 3.5)
project(ros2_python_extension)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(PYBIND11_PYTHON_VERSION 3)
set(PYBIND11_FINDPYTHON ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

add_library(ros2_python_extension INTERFACE)
target_include_directories(ros2_python_extension INTERFACE include)

ament_target_dependencies(ros2_python_extension INTERFACE
  rclcpp
  pybind11
  Python3
)

install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(rclcpp)
ament_export_dependencies(pybind11)
ament_export_dependencies(Python3)
ament_export_include_directories(include)

ament_package()
