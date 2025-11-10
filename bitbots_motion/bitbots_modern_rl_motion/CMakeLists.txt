cmake_minimum_required(VERSION 3.5)
project(bitbots_modern_rl_motion)

find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(rclpy REQUIRED)

enable_bitbots_docs()

ament_python_install_package(${PROJECT_NAME})

ament_package()
