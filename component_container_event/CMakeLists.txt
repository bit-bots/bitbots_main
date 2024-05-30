cmake_minimum_required(VERSION 3.8)
project(component_container_event)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(component_container_event src/component_container_event.cpp)

ament_target_dependencies(component_container_event rclcpp rclcpp_components)

# Install executables
install(TARGETS component_container_event
        RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
