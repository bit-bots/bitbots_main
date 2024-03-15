cmake_minimum_required(VERSION 3.5)
project(bitbots_utils)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

enable_bitbots_docs()

# Define include directories for cpp library headers
set(INCLUDE_DIRS include)
include_directories(${INCLUDE_DIRS})

add_compile_options(-Wall -Werror -Wno-unused -pedantic -Wextra -fPIC)

# Add cpp library
add_library(${PROJECT_NAME} src/utils.cpp)

# Add dependencies to cpp library
ament_target_dependencies(${PROJECT_NAME} rclcpp tf2_ros)

ament_export_dependencies(rclcpp)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})

# Python install
ament_python_install_package(${PROJECT_NAME})

# CPP Script install
add_executable(tf_delay_plot scripts/tf_delay_plot.cpp)

ament_target_dependencies(tf_delay_plot ament_cmake rclcpp tf2_msgs std_msgs)

install(TARGETS tf_delay_plot DESTINATION lib/${PROJECT_NAME})

# Copy config and launch files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts/ USE_SOURCE_PERMISSIONS
        DESTINATION lib/${PROJECT_NAME})

# Install library
install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

ament_package()
