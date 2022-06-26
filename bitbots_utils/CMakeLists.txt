cmake_minimum_required(VERSION 3.5)
project(bitbots_utils)

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

enable_bitbots_docs()

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION lib/${PROJECT_NAME})

ament_package()
