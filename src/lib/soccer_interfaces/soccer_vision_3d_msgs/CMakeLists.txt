cmake_minimum_required(VERSION 3.8)
project(soccer_vision_3d_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(soccer_vision_attribute_msgs REQUIRED)

# generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ball.msg"
  "msg/BallArray.msg"
  "msg/FieldBoundary.msg"
  "msg/Goalpost.msg"
  "msg/GoalpostArray.msg"
  "msg/MarkingEllipse.msg"
  "msg/MarkingIntersection.msg"
  "msg/MarkingSegment.msg"
  "msg/MarkingArray.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/Robot.msg"
  "msg/RobotArray.msg"
  DEPENDENCIES vision_msgs soccer_vision_attribute_msgs
)

# export dependencies
ament_export_dependencies(rosidl_default_runtime)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
