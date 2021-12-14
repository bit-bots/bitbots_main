cmake_minimum_required(VERSION 3.5)
project(bitbots_docs)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)

set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES})

file(COPY files
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

# we cannot use the compiled catkin package of bitbots_docs since that would require this pacakage to depend on itself.
# therefore we need to compile the sub cmake file ourselves and include it manualy
configure_file(cmake/enable_bitbots_docs.cmake.in
  ${CMAKE_BINARY_DIR}/enable_bitbots_docs.cmake @ONLY)
include(${CMAKE_BINARY_DIR}/enable_bitbots_docs.cmake)
enable_bitbots_docs()

ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
