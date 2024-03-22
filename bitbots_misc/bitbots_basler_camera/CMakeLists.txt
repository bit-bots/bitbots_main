cmake_minimum_required(VERSION 3.5)
project(bitbots_basler_camera)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pylon 7.1.0 REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(generate_parameter_library REQUIRED)

add_compile_options(-Wall -Werror -Wno-unused)

generate_parameter_library(
  pylon_camera_parameters # cmake target name for the parameter library
  config/camera_settings.yaml)

add_executable(postprocessing src/postprocessing.cpp)

target_link_libraries(postprocessing ${OpenCV_LIBRARIES} pylon::pylon
                      pylon_camera_parameters)

ament_target_dependencies(
  postprocessing
  ament_cmake
  bitbots_docs
  cv_bridge
  image_transport
  rclcpp
  sensor_msgs
  camera_info_manager
  generate_parameter_library
  OpenCV)

enable_bitbots_docs()

install(TARGETS postprocessing DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
