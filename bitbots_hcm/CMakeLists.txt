cmake_minimum_required(VERSION 3.5)
project(bitbots_hcm)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(rclpy REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

ament_export_dependencies(rclpy)
ament_export_dependencies(humanoid_league_msgs)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(ament_cmake)

install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
ament_package()
