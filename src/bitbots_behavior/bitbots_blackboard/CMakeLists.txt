cmake_minimum_required(VERSION 3.5)
project(bitbots_blackboard)

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

enable_bitbots_docs()

if(BUILD_TESTING)
  find_package(ament_cmake_mypy REQUIRED)
  ament_mypy(CONFIG_FILE "${CMAKE_CURRENT_LIST_DIR}/mypy.ini")
endif()

ament_python_install_package(${PROJECT_NAME})

ament_package()
