cmake_minimum_required(VERSION 3.5)
project(bitbots_docs)

find_package(ament_cmake REQUIRED)

# the good way would be to install the files there instead of copying them
# but then, we can't build the documentation in this package
file(COPY files DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

# we cannot use the compiled catkin package of bitbots_docs since that would require this package to depend on itself.
# therefore we need to compile the sub cmake file ourselves and include it manualy
configure_file(cmake/enable_bitbots_docs.cmake.in
  ${CMAKE_BINARY_DIR}/enable_bitbots_docs.cmake @ONLY)
include(${CMAKE_BINARY_DIR}/enable_bitbots_docs.cmake)
enable_bitbots_docs()

ament_package(CONFIG_EXTRAS cmake/enable_bitbots_docs.cmake.in)
