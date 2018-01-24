cmake_minimum_required(VERSION 2.8)
project(bitbots_communication)

find_package(roscpp REQUIRED)
catkin_package()

file(GLOB SOURCES src/bitbots_communication/*.cpp)

include_directories(${catkin_INCLUDE_DIRS} include/bitbots_communication ${roscpp_INCLUDE_DIRS})
add_executable(robocom ${SOURCES})
target_link_libraries(robocom ${roscpp_LIBRARIES} ${catkin_LIBRARIES})