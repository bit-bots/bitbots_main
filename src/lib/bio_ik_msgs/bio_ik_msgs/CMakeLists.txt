cmake_minimum_required(VERSION 3.5)
project(bio_ik_msgs)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AvoidJointLimitsGoal.msg"
  "msg/BalanceGoal.msg"
  "msg/CenterJointsGoal.msg"
  "msg/ConeGoal.msg"
  "msg/DirectionGoal.msg"
  "msg/IKRequest.msg"
  "msg/IKResponse.msg"
  "msg/JointVariableGoal.msg"
  "msg/LineGoal.msg"
  "msg/LookAtGoal.msg"
  "msg/MaxDistanceGoal.msg"
  "msg/MinDistanceGoal.msg"
  "msg/MinimalDisplacementGoal.msg"
  "msg/OrientationGoal.msg"
  "msg/PoseGoal.msg"
  "msg/PositionGoal.msg"
  "msg/SideGoal.msg"
  "msg/TouchGoal.msg"
  "srv/GetIK.srv"
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  std_msgs
  moveit_msgs
  builtin_interfaces)

ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(std_msgs)
ament_export_dependencies(moveit_msgs)
ament_export_dependencies(ament_cmake)

ament_package()
