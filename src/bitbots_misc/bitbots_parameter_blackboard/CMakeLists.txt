cmake_minimum_required(VERSION 3.5)
project(bitbots_parameter_blackboard)

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)

enable_bitbots_docs()

# Copy config and launch files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
