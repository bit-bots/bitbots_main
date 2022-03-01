cmake_minimum_required(VERSION 3.5)
project(bitbots_splines)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(bio_ik REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(fmt REQUIRED)
find_package(fp REQUIRED)
find_package(range-v3 REQUIRED)

set(INCLUDE_DIRS include)
include_directories(${INCLUDE_DIRS})

add_compile_options(-Wall -Werror -Wno-unused)

enable_bitbots_docs()

set(SOURCES src/Spline/polynom.cpp
  src/Spline/smooth_spline.cpp
  src/Spline/spline.cpp
  src/Spline/pose_spline.cpp
  src/Spline/position_spline.cpp
  src/Utils/newton_binomial.cpp
  src/Utils/combination.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

ament_target_dependencies(${PROJECT_NAME}
    ament_cmake
    bitbots_docs
    bio_ik
    rclcpp
    std_msgs
    geometry_msgs
    tf2_geometry_msgs
    moveit_core
    moveit_ros_planning
    Eigen3
    PkgConfig
    fmt
    fp
    range-v3
)

install(DIRECTORY include/
    DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(bio_ik)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})

ament_package()
