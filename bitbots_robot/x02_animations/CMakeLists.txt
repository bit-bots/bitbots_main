cmake_minimum_required(VERSION 3.5)
project(x02_animations)

find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)

enable_bitbots_docs()

install(DIRECTORY animations DESTINATION share/${PROJECT_NAME})

ament_package()
