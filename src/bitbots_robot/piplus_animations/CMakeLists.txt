cmake_minimum_required(VERSION 3.5)
project(piplus_animations)

find_package(ament_cmake REQUIRED)


install(DIRECTORY animations DESTINATION share/${PROJECT_NAME})

ament_package()
