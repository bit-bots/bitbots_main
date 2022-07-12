cmake_minimum_required(VERSION 3.5)
project(bitbots_buttons)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(test_msgs REQUIRED)

add_executable(button_node src/button_node.cpp)
ament_target_dependencies(button_node
        ament_cmake
        backward_ros
        bitbots_msgs
        humanoid_league_msgs
        rclcpp
        std_msgs
        std_srvs
        test_msgs)

#enable_bitbots_docs()


install(TARGETS button_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(std_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(bitbots_msgs)
ament_export_dependencies(humanoid_league_msgs)
ament_export_dependencies(std_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
