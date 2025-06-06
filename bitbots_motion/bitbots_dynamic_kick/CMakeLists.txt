cmake_minimum_required(VERSION 3.5)
project(bitbots_dynamic_kick)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Build with release optimizations and debug symbols by default
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebug)
endif()

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(bitbots_splines REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(Eigen3 REQUIRED)
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

find_package(Python3 COMPONENTS Interpreter Development)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/KickDebug.msg" DEPENDENCIES
                           std_msgs geometry_msgs Python3)

set(INCLUDE_DIRS include ${PYTHON_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

add_compile_options(-Wall -Werror -Wno-unused)

set(SOURCES src/kick_node.cpp src/kick_engine.cpp src/stabilizer.cpp
            src/visualizer.cpp src/kick_ik.cpp)

add_executable(KickNode ${SOURCES})

ament_target_dependencies(
  KickNode
  ament_cmake
  biped_interfaces
  bitbots_msgs
  bitbots_splines
  control_toolbox
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
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

target_link_libraries(KickNode "${cpp_typesupport_target}")

# create the kick python wrapper

# pybind11_add_module(py_dynamic_kick SHARED src/walk_pywrapper.cpp ${SOURCES})
# ament_target_dependencies(py_dynamic_kick PUBLIC ament_cmake biped_interfaces
# bitbots_msgs bitbots_splines control_toolbox geometry_msgs
# moveit_ros_planning_interface rclcpp ros2_python_extension rot_conv
# sensor_msgs std_msgs tf2 tf2_eigen tf2_geometry_msgs tf2_ros)
# target_link_libraries(py_dynamic_kick PRIVATE "${cpp_typesupport_target}")

# ament_python_install_package(bitbots_dynamic_kick_py)

# install(TARGETS py_dynamic_kick DESTINATION
# "${PYTHON_INSTALL_DIR}/bitbots_quintic_walk_py" )

enable_bitbots_docs()

install(DIRECTORY scripts USE_SOURCE_PERMISSIONS
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS KickNode DESTINATION lib/${PROJECT_NAME})

ament_package()
