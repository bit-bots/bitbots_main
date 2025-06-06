cmake_minimum_required(VERSION 3.12)
project(bitbots_hcm)

# Build with release optimizations and debug symbols by default
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebug)
endif()

set(PYBIND11_PYTHON_VERSION 3)
set(PYBIND11_FINDPYTHON ON)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(backward_ros REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(rclpy REQUIRED)
find_package(ros2_python_extension REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fvisibility=hidden -Wall -Wextra -Wpedantic)
endif()

include_directories(${PYTHON_INCLUDE_DIRS})
set(SOURCES src/hcm.cpp)

add_executable(HCM ${SOURCES})

ament_target_dependencies(
  HCM
  ament_cmake
  backward_ros
  bitbots_msgs
  builtin_interfaces
  geometry_msgs
  rclcpp
  ros2_python_extension
  sensor_msgs
  std_msgs)

target_link_libraries(HCM Python3::Python)

enable_bitbots_docs()

install(TARGETS HCM DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts/ USE_SOURCE_PERMISSIONS
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    hcm_py_test test/pytest PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}" APPEND_ENV
    PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR})

  find_package(ament_cmake_mypy REQUIRED)
  ament_mypy(CONFIG_FILE "${CMAKE_CURRENT_LIST_DIR}/mypy.ini")
endif()

ament_python_install_package(${PROJECT_NAME})
ament_package()
