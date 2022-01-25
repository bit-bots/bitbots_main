cmake_minimum_required(VERSION 3.5)
project(bitbots_convenience_frames)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclpy REQUIRED)

set(INCLUDE_DIRS include ${geometry_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS} ${bitbots_msgs_INCLUDE_DIRS}
  ${humanoid_league_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS} ${bitbots_docs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS} ${rclpy_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${geometry_msgs_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS} ${tf2_ros_LIBRARY_DIRS} ${bitbots_msgs_LIBRARY_DIRS}
  ${humanoid_league_msgs_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS}
  ${tf2_geometry_msgs_LIBRARY_DIRS} ${bitbots_docs_LIBRARY_DIRS}
  ${tf2_LIBRARY_DIRS} ${rclpy_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${geometry_msgs_LIBRARIES} ${rclcpp_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${tf2_ros_LIBRARIES} ${bitbots_msgs_LIBRARIES} ${humanoid_league_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES} ${bitbots_docs_LIBRARIES}
  ${tf2_LIBRARIES} ${rclpy_LIBRARIES})

add_compile_options(-Wall -Werror -Wno-unused)

## Declare a C++ executable
add_executable(convenience_frames src/convenience_frames.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(convenience_frames ${LIBS})

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(bitbots_msgs)
ament_export_dependencies(humanoid_league_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(tf2)
ament_export_dependencies(rclpy)
ament_export_include_directories(${INCLUDE_DIRS})


install(TARGETS convenience_frames
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})


ament_package()
