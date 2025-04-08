cmake_minimum_required(VERSION 3.5)
project(bitbots_robot_description)

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)

enable_bitbots_docs()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
