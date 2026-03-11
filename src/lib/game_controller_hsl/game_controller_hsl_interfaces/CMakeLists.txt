cmake_minimum_required(VERSION 3.5)
project(game_controller_hsl_interfaces)

find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GameState.msg"
  "msg/PlayerStatusPose.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

ament_package()
