cmake_minimum_required(VERSION 3.0)
project(bitbots_test VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS bitbots_docs roscpp rostest)

catkin_python_setup()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp
        CFG_EXTRAS enable_bitbots_tests.cmake
)

include_directories(include ${catkin_INCLUDE_DIRS})
set(SOURCES src/bitbots_test.cpp)
add_library(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIBR_DESTINATION})

enable_bitbots_docs()

if (CATKIN_ENABLE_TESTING)
    find_package(Boost REQUIRED)
    include_directories(${Boost_INCLUDE_DIRS})
    # we cannot use the compiled catkin package of bitbots_test since that would require this package to depend on itself
    # therefore we need to compile the sub cmake file ourselves and include it manually
    configure_file(cmake/enable_bitbots_tests.cmake.in ${CMAKE_BINARY_DIR}/enable_bitbots_tests.cmake @ONLY)
    include(${CMAKE_BINARY_DIR}/enable_bitbots_tests.cmake)
    enable_bitbots_tests(CXX_SOURCES "${SOURCES}")
endif()
