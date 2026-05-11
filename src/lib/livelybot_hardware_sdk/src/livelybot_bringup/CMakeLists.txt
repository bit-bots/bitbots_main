cmake_minimum_required(VERSION 3.8)
project(livelybot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livelybot_serial REQUIRED)

set(TOOL_NODES
  canboard_update
  motor_feedback
  motor_move_zero
  motor_rerun
  motor_set_zero
  motor_timeout
)

foreach(target ${TOOL_NODES})
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} rclcpp sensor_msgs std_msgs livelybot_serial)
  install(TARGETS ${target}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
