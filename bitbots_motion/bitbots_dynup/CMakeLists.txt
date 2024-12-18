cmake_minimum_required(VERSION 3.5)
project(bitbots_dynup)

# Add support for C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(PYBIND11_PYTHON_VERSION 3)
set(PYBIND11_FINDPYTHON ON)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(bio_ik REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(bitbots_splines REQUIRED)
find_package(bitbots_utils REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rot_conv REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(ros2_python_extension REQUIRED)

generate_parameter_library(dynup_parameters config/dynup_config.yaml)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/DynupEngineDebug.msg"
  "msg/DynupPoses.msg"
  "msg/DynupIkOffset.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs)

include_directories(include ${PYTHON_INCLUDE_DIRS})

add_compile_options(-Wall -Werror -Wno-unused -fPIC)

set(SOURCES
    src/dynup_engine.cpp
    src/dynup_ik.cpp
    src/dynup_node.cpp
    src/dynup_pywrapper.cpp
    src/dynup_stabilizer.cpp
    src/visualizer.cpp
    src/dynup_utils.cpp)

add_executable(DynupNode ${SOURCES})

ament_target_dependencies(
  DynupNode
  ament_cmake
  bio_ik
  bitbots_msgs
  bitbots_splines
  bitbots_utils
  control_msgs
  control_toolbox
  generate_parameter_library
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  ros2_python_extension
  rot_conv
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  Eigen3)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")

target_link_libraries(DynupNode "${cpp_typesupport_target}" rclcpp::rclcpp
                      dynup_parameters)
# create the lib
pybind11_add_module(libpy_dynup SHARED src/dynup_pywrapper.cpp ${SOURCES})
ament_target_dependencies(
  libpy_dynup
  PUBLIC
  ament_cmake
  bio_ik
  bitbots_msgs
  bitbots_splines
  bitbots_utils
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  ros2_python_extension
  rot_conv
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros)

target_link_libraries(libpy_dynup PRIVATE "${cpp_typesupport_target}"
                                          dynup_parameters)

ament_python_install_package(bitbots_dynup_py)

install(TARGETS libpy_dynup
        DESTINATION "${PYTHON_INSTALL_DIR}/bitbots_dynup_py")

enable_bitbots_docs()

install(TARGETS DynupNode DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts USE_SOURCE_PERMISSIONS
        DESTINATION lib/${PROJECT_NAME})

ament_package()
