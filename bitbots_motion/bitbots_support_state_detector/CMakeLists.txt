cmake_minimum_required(VERSION 3.5)
project(bitbots_support_state_detector)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(bitbots_utils REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)

add_compile_options(-Wall -Werror -Wno-unused)

generate_parameter_library(
  support_state_detector_parameters
  config/support_state_detector_template.yaml
)

# Include
include_directories(include)

add_executable(walk_support_state_detector src/walk_support_state_detector.cpp)

target_link_libraries(
  walk_support_state_detector 
  rclcpp::rclcpp
  support_state_detector_parameters)

ament_target_dependencies(
  walk_support_state_detector
  biped_interfaces
  bitbots_msgs
  generate_parameter_library
  std_msgs
)

enable_bitbots_docs()

install(TARGETS walk_support_state_detector DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
