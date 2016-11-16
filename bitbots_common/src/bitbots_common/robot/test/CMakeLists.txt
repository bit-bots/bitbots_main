cmake_minimum_required(VERSION 2.6)

FILE(GLOB TEST_SRC *.cpp)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(.)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${gtest_c_flags} -pedantic")

if(${YAMLPATH} MATCHES YAMLPATH-NOTFOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DYAML_NOTFOUND")
endif()

set(TEST_EXECUTABLES "")
foreach(T ${TEST_SRC})
    string(REGEX REPLACE ".*/" "" T ${T})
    string(REGEX REPLACE ".cpp" "" T ${T})
    add_executable(${T}.x ${T}.cpp)
    target_link_libraries(${T}.x robot)
    target_link_libraries(${T}.x ${gtest_link_flags})

    add_test(${T} ${T}.x)
    set(TEST_EXECUTABLES ${TEST_EXECUTABLES} ${T}.x)
endforeach()

install(
    TARGETS ${TEST_EXECUTABLES}
    RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/lib/bitbots/robot/test
)

install(
    TARGETS ${TEST_EXECUTABLES}
    RUNTIME DESTINATION ${TEST_INSTALL_DIR}
)
