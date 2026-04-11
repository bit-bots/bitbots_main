cmake_minimum_required(VERSION 3.8)
project(livelybot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY robot_param/
  DESTINATION share/${PROJECT_NAME}/robot_param
)

ament_package()
