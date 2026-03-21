cmake_minimum_required(VERSION 3.8)
project(soccer_geometry_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointWithCovariance.msg"
  DEPENDENCIES geometry_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()
