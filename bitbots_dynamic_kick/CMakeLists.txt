cmake_minimum_required(VERSION 2.8.3)
project(bitbots_dynamic_kick)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
)

set(SOURCES
    src/KickNode.cpp
    src/KickEngine.cpp
    src/Stabilizer.cpp
)

add_executable(KickNode ${SOURCES})

target_link_libraries(KickNode ${catkin_LIBRARIES})
