cmake_minimum_required(VERSION 3.5)
project(wolfgang_description)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(bitbots_docs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)

set(INCLUDE_DIRS ${bitbots_docs_INCLUDE_DIRS} ${urdf_INCLUDE_DIRS}
  ${xacro_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${bitbots_docs_LIBRARY_DIRS} ${urdf_LIBRARY_DIRS}
  ${xacro_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS}
  ${tf2_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${bitbots_docs_LIBRARIES} ${urdf_LIBRARIES} ${xacro_LIBRARIES}
  ${sensor_msgs_LIBRARIES} ${ament_cmake_LIBRARIES} ${tf2_LIBRARIES}
  ${rclcpp_LIBRARIES})

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

ament_export_dependencies(bitbots_docs)
ament_export_dependencies(urdf)
ament_export_dependencies(xacro)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(tf2)
ament_export_dependencies(rclcpp)
ament_export_include_directories(${INCLUDE_DIRS})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})


ament_package()
