cmake_minimum_required(VERSION 3.5)
project(bitbots_hcm)

set(PYBIND11_PYTHON_VERSION 3)
set(PYBIND11_FINDPYTHON ON)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(backward_ros REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_python_extension REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})
set(SOURCES src/hcm.cpp)

add_executable(HCM ${SOURCES})

ament_target_dependencies(HCM
    ament_cmake
    backward_ros
    bitbots_msgs
    builtin_interfaces
    geometry_msgs
    humanoid_league_msgs
    rclcpp
    ros2_python_extension
    sensor_msgs
    std_msgs)

target_link_libraries(HCM ${PYTHON_LIBRARIES})

enable_bitbots_docs()

install(TARGETS HCM
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
ament_package()
