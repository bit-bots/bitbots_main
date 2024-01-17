cmake_minimum_required(VERSION 3.5)
project(bitbots_ceiling_cam)

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)

enable_bitbots_docs()

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
