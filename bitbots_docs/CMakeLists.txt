cmake_minimum_required(VERSION 2.8.3)
project(bitbots_docs)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS enable_bitbots_docs.cmake)
file(COPY files DESTINATION ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})

# we cannot use the compiled catkin package of bitbots_docs since that would require this pacakage to depend on itself.
# therefore we need to compile the sub cmake file ourselves and include it manualy
configure_file(cmake/enable_bitbots_docs.cmake.in ${CMAKE_BINARY_DIR}/enable_bitbots_docs.cmake @ONLY)
include(${CMAKE_BINARY_DIR}/enable_bitbots_docs.cmake)
enable_bitbots_docs()

