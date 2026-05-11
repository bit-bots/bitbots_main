cmake_minimum_required(VERSION 3.8)
project(soccer_model_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(soccer_geometry_msgs REQUIRED)
find_package(soccer_vision_attribute_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  std_msgs
  soccer_geometry_msgs
  soccer_vision_attribute_msgs)

# Generate Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ball.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/Robot.msg"
  "msg/RobotArray.msg"
  DEPENDENCIES ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()
