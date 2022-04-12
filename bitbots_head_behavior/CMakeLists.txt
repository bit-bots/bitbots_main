cmake_minimum_required(VERSION 3.5)
project(bitbots_head_behavior)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(moveit_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(moveit_ros_robot_interaction REQUIRED)

set(INCLUDE_DIRS include ${moveit_core_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${moveit_ros_move_group_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
  ${bitbots_docs_INCLUDE_DIRS} ${moveit_ros_robot_interaction_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${moveit_core_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS}
  ${moveit_ros_move_group_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS}
  ${bitbots_docs_LIBRARY_DIRS} ${moveit_ros_robot_interaction_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${moveit_core_LIBRARIES} ${std_msgs_LIBRARIES}
  ${moveit_ros_move_group_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${bitbots_docs_LIBRARIES} ${moveit_ros_robot_interaction_LIBRARIES})

add_compile_options(-Wall -Werror -Wno-unused)

find_package(Eigen3 REQUIRED)
find_package(PythonLibs)
if(PYTHONLIBS_VERSION_STRING VERSION_LESS "3.8")
  find_package(Boost REQUIRED COMPONENTS python3)
else()
  find_package(Boost REQUIRED COMPONENTS python)
endif()

find_package(PythonLibs COMPONENTS Interpreter Development)

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

# create the lib
add_library(collision_checker SHARED src/collision_checker.cpp)

# link
target_link_libraries(collision_checker ${LIBS} ${Boost_LIBRARIES}
  ${CODE_LIBRARIES})

set_target_properties(collision_checker
  PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY
  ${CATKIN_PACKAGE_PYTHON_DESTINATION})

ament_export_dependencies(moveit_core)
ament_export_dependencies(std_msgs)
ament_export_dependencies(moveit_ros_move_group)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(moveit_ros_robot_interaction)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(collision_checker ${LIBS})


ament_package()
