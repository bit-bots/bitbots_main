cmake_minimum_required(VERSION 3.10)
project(audio_common)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_library(PORTAUDIO_LIB portaudio REQUIRED)

include_directories(
  include
  ${PORTAUDIO_INCLUDE_DIR}
)

# audio_capturer__node
add_executable(audio_capturer_node
  src/audio_common/audio_capturer_node.cpp
  src/audio_capturer_main.cpp
)
target_link_libraries(audio_capturer_node
  ${audio_common_msgs_TARGETS}
  ${PORTAUDIO_LIB}
  rclcpp::rclcpp
)
install(TARGETS
  audio_capturer_node
  DESTINATION lib/${PROJECT_NAME}
)

# audio_player_node
add_executable(audio_player_node
  src/audio_common/audio_player_node.cpp
  src/audio_player_main.cpp
)
target_link_libraries(audio_player_node
  ${audio_common_msgs_TARGETS}
  ${PORTAUDIO_LIB}
  rclcpp::rclcpp
)
install(TARGETS
  audio_player_node
  DESTINATION lib/${PROJECT_NAME}
)

# music_node
add_executable(music_node
  src/audio_common/wave_file.cpp
  src/audio_common/music_node.cpp
  src/music_main.cpp
)
target_link_libraries(music_node
  ament_index_cpp::ament_index_cpp
  ${audio_common_msgs_TARGETS}
  ${PORTAUDIO_LIB}
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
)

install(TARGETS
  music_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  samples
  DESTINATION share/${PROJECT_NAME}
)

# tts node
add_executable(tts_node
  src/audio_common/wave_file.cpp
  src/audio_common/tts_node.cpp
  src/tts_main.cpp
)
target_link_libraries(tts_node
  ${audio_common_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${PORTAUDIO_LIB}
)
install(TARGETS
  tts_node
  DESTINATION lib/${PROJECT_NAME}
)

# Create lib
set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(SOURCES
  src/audio_common/wave_file.cpp
)

add_library(${LIB} STATIC ${SOURCES})
target_link_libraries(${LIB} ${DEPENDENCIES})

install(TARGETS ${LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE .clang-format)
endif()

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)
ament_export_libraries(${LIB})

ament_package()
