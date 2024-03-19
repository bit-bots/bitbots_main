cmake_minimum_required(VERSION 3.5)
project(bitbots_basler_camera)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(backward_ros REQUIRED)
find_package(OpenCV REQUIRED)

add_compile_options(-Wall -Werror -Wno-unused)

add_executable(postprocessing src/postprocessing.cpp)

target_link_libraries(postprocessing
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(
  postprocessing
  ament_cmake
  bitbots_docs
  rclcpp)

enable_bitbots_docs()

install(TARGETS postprocessing DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
