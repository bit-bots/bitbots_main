cmake_minimum_required(VERSION 2.8.3)
project(system_monitor)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  Cpu.msg
  Filesystem.msg
  NetworkInterface.msg
  Workload.msg
)

generate_messages (
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)
