cmake_minimum_required(VERSION 3.5)
project(bitbots_odometry)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(rotconv REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

set(INCLUDE_DIRS include ${tf2_eigen_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS} ${message_filters_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS} ${bitbots_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} ${bitbots_docs_INCLUDE_DIRS}
  ${rotconv_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${tf2_eigen_LIBRARY_DIRS} ${tf2_LIBRARY_DIRS}
  ${nav_msgs_LIBRARY_DIRS} ${message_filters_LIBRARY_DIRS}
  ${tf2_geometry_msgs_LIBRARY_DIRS} ${bitbots_msgs_LIBRARY_DIRS}
  ${geometry_msgs_LIBRARY_DIRS} ${bitbots_docs_LIBRARY_DIRS}
  ${rotconv_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS} ${tf2_ros_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${tf2_eigen_LIBRARIES} ${tf2_LIBRARIES} ${nav_msgs_LIBRARIES}
  ${message_filters_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES}
  ${bitbots_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${bitbots_docs_LIBRARIES}
  ${rotconv_LIBRARIES} ${sensor_msgs_LIBRARIES} ${tf2_ros_LIBRARIES}
  ${ament_cmake_LIBRARIES} ${rclcpp_LIBRARIES})

add_compile_options(-Wall -Werror -Wno-unused)

add_executable(odometry_fuser src/odometry_fuser.cpp)
add_executable(motion_odometry src/motion_odometry.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(odometry_fuser ${LIBS} ${Eigen3_LIBRARIES})

target_link_libraries(motion_odometry ${LIBS})

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

ament_export_dependencies(tf2_eigen)
ament_export_dependencies(tf2)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(message_filters)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(bitbots_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(rotconv)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
