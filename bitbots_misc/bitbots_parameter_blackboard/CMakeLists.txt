cmake_minimum_required(VERSION 3.5)
project(bitbots_parameter_blackboard)

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

enable_bitbots_docs()

# Python install
ament_python_install_package(${PROJECT_NAME})

# Copy config and launch files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
