cmake_minimum_required(VERSION 3.5)
project(bitbots_rl_motion)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)

set(INCLUDE_DIRS ${rclpy_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS} ${bitbots_docs_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${rclpy_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS} ${bitbots_docs_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${rclpy_LIBRARIES} ${std_msgs_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${bitbots_docs_LIBRARIES})

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(bitbots_docs)
ament_export_include_directories(${INCLUDE_DIRS})

install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/rl_walk.py
  DESTINATION lib/${PROJECT_NAME} )

ament_package()
