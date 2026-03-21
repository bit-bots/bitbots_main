cmake_minimum_required(VERSION 3.9)
project(bitbots_tf_buffer)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# Include CheckIPOSupported module
include(CheckIPOSupported)

# Check if IPO is supported
check_ipo_supported(RESULT ipo_supported)

if(ipo_supported)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
  # Set the number of parallel LTO jobs
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto=4")
endif()


set(PYBIND11_PYTHON_VERSION 3)
set(PYBIND11_FINDPYTHON ON)
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_python_extension REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_compile_options(-Wall -Wno-unused)

pybind11_add_module(cpp_wrapper SHARED src/bitbots_tf_buffer.cpp)

ament_target_dependencies(
  cpp_wrapper
  PUBLIC
  rclcpp
  rcl
  tf2
  tf2_ros
  ros2_python_extension)

target_link_libraries(cpp_wrapper PRIVATE pybind11::module)

ament_python_install_package(${PROJECT_NAME})

ament_get_python_install_dir(PYTHON_INSTALL_DIR)

install(TARGETS cpp_wrapper DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

ament_package()
