cmake_minimum_required(VERSION 3.5)
project(bitbots_ipm)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/global_parameter_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME global_parameter_bridge
)

ament_package()
