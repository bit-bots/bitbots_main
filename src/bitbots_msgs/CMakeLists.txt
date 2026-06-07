cmake_minimum_required(VERSION 3.5)
project(bitbots_msgs)

# Build with release optimizations and debug symbols by default
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebug)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "action/Kick.action"
  "action/LookAt.action"
  "action/PlayAnimation.action"
  "msg/Animation.msg"
  "msg/Buttons.msg"
  "msg/Cpu.msg"
  "msg/Filesystem.msg"
  "msg/HeadMode.msg"
  "msg/JointCommand.msg"
  "msg/JointTorque.msg"
  "msg/ModelStates.msg"
  "msg/NetworkInterface.msg"
  "msg/PoseWithCertainty.msg"
  "msg/PoseWithCertaintyArray.msg"
  "msg/RobotControlState.msg"
  "msg/RobotRelative.msg"
  "msg/RobotRelativeArray.msg"
  "msg/Strategy.msg"
  "msg/TeamData.msg"
  "msg/TTS.msg"
  "msg/Workload.msg"
  "srv/AddAnimation.srv"
  "srv/ManualPenalize.srv"
  "srv/SetObjectPose.srv"
  "srv/SetObjectPosition.srv"
  "srv/SetTeachingMode.srv"
  "srv/SimulatorPush.srv"
  DEPENDENCIES
  action_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs)

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

ament_package()
