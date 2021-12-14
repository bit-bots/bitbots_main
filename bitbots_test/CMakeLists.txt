cmake_minimum_required(VERSION 3.5)
project(bitbots_test VERSION 0.0.1)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(rostest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bitbots_docs REQUIRED)

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS} ${rostest_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS} ${bitbots_docs_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS} ${rostest_LIBRARY_DIRS}
  ${rclcpp_LIBRARY_DIRS} ${bitbots_docs_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES} ${rostest_LIBRARIES} ${rclcpp_LIBRARIES}
  ${bitbots_docs_LIBRARIES})

set(CMAKE_CXX_STANDARD 17)

set(SOURCES src/bitbots_test.cpp)
add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIBS})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION bin)

enable_bitbots_docs()

if(BUILD_TESTING)
  find_package(Boost REQUIRED)
  # we cannot use the compiled catkin package of bitbots_test since that would require this package to depend on itself
  # therefore we need to compile the sub cmake file ourselves and include it manually
  configure_file(cmake/enable_bitbots_tests.cmake.in
    ${CMAKE_BINARY_DIR}/enable_bitbots_tests.cmake @ONLY)
  include(${CMAKE_BINARY_DIR}/enable_bitbots_tests.cmake)
  enable_bitbots_tests(CXX_SOURCES "${SOURCES}")
endif()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rostest)
ament_export_dependencies(rclcpp)
ament_export_dependencies(bitbots_docs)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS})

ament_package()
