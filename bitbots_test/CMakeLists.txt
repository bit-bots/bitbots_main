cmake_minimum_required(VERSION 3.0)
project(bitbots_test VERSION 0.0.1)

find_package(catkin REQUIRED COMPONENTS bitbots_docs)

catkin_python_setup()

enable_bitbots_docs()

catkin_package(CFG_EXTRAS enable_bitbots_tests.cmake)

# we cannot use the compiled catkin package of bitbots_test since that would require this package to depend on itself
# therefore we need to compile the sub cmake file ourselves and include it manually
configure_file(cmake/enable_bitbots_tests.cmake.in ${CMAKE_BINARY_DIR}/enable_bitbots_tests.cmake @ONLY)
include(${CMAKE_BINARY_DIR}/enable_bitbots_tests.cmake)
enable_bitbots_tests()
