cmake_minimum_required(VERSION 3.5)
project(bitbots_utils)

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)


set(INCLUDE_DIRS include)
include_directories(${INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Werror -Wno-unused -pedantic -Wextra)

enable_bitbots_docs()

add_executable(tf_delay_plot scripts/tf_delay_plot.cpp)

ament_target_dependencies(tf_delay_plot
        ament_cmake
        rclcpp
        tf2_msgs
        std_msgs)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS tf_delay_plot
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION lib/${PROJECT_NAME})

ament_package()
