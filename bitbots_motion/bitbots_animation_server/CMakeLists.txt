cmake_minimum_required(VERSION 3.5)
project(bitbots_animation_server)

find_package(bitbots_docs REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

enable_bitbots_docs()

ament_python_install_package(${PROJECT_NAME})

ament_package()
