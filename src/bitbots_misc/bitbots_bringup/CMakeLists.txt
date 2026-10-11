cmake_minimum_required(VERSION 3.5)
project(bitbots_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
