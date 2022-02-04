cmake_minimum_required(VERSION 3.5)
project(bitbots_moveit_bindings)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)

set(INCLUDE_DIRS ${bitbots_docs_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS} ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${bitbots_docs_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS}
  ${moveit_core_LIBRARY_DIRS} ${moveit_ros_planning_interface_LIBRARY_DIRS}
  ${moveit_ros_planning_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${bitbots_docs_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${moveit_core_LIBRARIES} ${moveit_ros_planning_interface_LIBRARIES}
  ${moveit_ros_planning_LIBRARIES})

add_compile_options(-Wall -Werror -Wno-unused)

find_package(PythonLibs COMPONENTS Interpreter Development)
if(PYTHONLIBS_VERSION_STRING VERSION_LESS "3.8")
  find_package(Boost REQUIRED COMPONENTS python3)
else()
  find_package(Boost REQUIRED COMPONENTS python)
endif()

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

add_library(bitbots_moveit_bindings SHARED src/bitbots_moveit_bindings.cpp)
target_include_directories(bitbots_moveit_bindings
  PRIVATE ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
target_link_libraries(bitbots_moveit_bindings ${LIBS} ${Boost_LIBRARIES})
set_target_properties(bitbots_moveit_bindings
  PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY
  ${CATKIN_PACKAGE_PYTHON_DESTINATION})

ament_export_dependencies(bitbots_docs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_ros_planning_interface)
ament_export_dependencies(moveit_ros_planning)
ament_export_libraries(bitbots_moveit_bindings ${LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})

ament_package()
