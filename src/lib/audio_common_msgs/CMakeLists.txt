cmake_minimum_required(VERSION 3.10)
project(audio_common_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioData.msg"
  "msg/AudioInfo.msg"
  "msg/Audio.msg"
  "msg/AudioStamped.msg"
  "action/TTS.action"
  "srv/MusicPlay.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(
  std_msgs
  rosidl_default_runtime
)

ament_package()
