cmake_minimum_required(VERSION 3.5)
project(wolfgang_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)

enable_bitbots_docs()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
