cmake_minimum_required(VERSION 3.5)
project(bitbots_robot_description)

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)

set(INCLUDE_DIRS include)
include_directories(${INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Werror -Wno-unused -pedantic -Wextra)

enable_bitbots_docs()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
