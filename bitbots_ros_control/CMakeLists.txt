cmake_minimum_required(VERSION 3.5)
project(bitbots_ros_control)

# Add support for C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(bitbots_buttons REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(yaml-cpp REQUIRED)


set(INCLUDE_DIRS include)
include_directories(${INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Werror -Wno-unused -pedantic -Wextra)


set(SOURCES
        src/bitfoot_hardware_interface.cpp
        src/button_hardware_interface.cpp
        src/core_hardware_interface.cpp
        src/dynamixel_servo_hardware_interface.cpp
        src/imu_hardware_interface.cpp
        src/leds_hardware_interface.cpp
        src/node.cpp
        src/servo_bus_interface.cpp
        src/utils.cpp
        src/wolfgang_hardware_interface.cpp include/bitbots_ros_control/hardware_interface.h)

enable_bitbots_docs()

add_executable(node ${SOURCES})
ament_target_dependencies(node
        ament_cmake
        backward_ros
        bitbots_buttons
        bitbots_docs
        bitbots_msgs
        controller_interface
        controller_manager
        diagnostic_msgs
        dynamixel_workbench_toolbox
        hardware_interface
        humanoid_league_msgs
        pluginlib
        rclcpp
        realtime_tools
        rosidl_typesupport_cpp
        std_msgs
        std_srvs
        tf2_ros
        transmission_interface
        yaml-cpp)

add_executable(pressure_converter src/pressure_converter.cpp)
ament_target_dependencies(pressure_converter
        ament_cmake
        backward_ros
        bitbots_buttons
        bitbots_docs
        bitbots_msgs
        controller_interface
        controller_manager
        diagnostic_msgs
        dynamixel_workbench_toolbox
        hardware_interface
        humanoid_league_msgs
        pluginlib
        rclcpp
        realtime_tools
        rosidl_typesupport_cpp
        std_msgs
        std_srvs
        tf2_ros
        transmission_interface
        yaml-cpp)
target_link_libraries(pressure_converter yaml-cpp)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(bitbots_buttons)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(bitbots_msgs)
ament_export_dependencies(controller_interface)
ament_export_dependencies(controller_manager)
ament_export_dependencies(dynamixel_workbench_toolbox)
ament_export_dependencies(hardware_interface)
ament_export_dependencies(humanoid_league_msgs)
ament_export_dependencies(pluginlib)
ament_export_dependencies(realtime_tools)
ament_export_dependencies(std_msgs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(transmission_interface)

install(TARGETS node
        DESTINATION lib/${PROJECT_NAME})
install(TARGETS pressure_converter
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts/ USE_SOURCE_PERMISSIONS DESTINATION lib/${PROJECT_NAME})

ament_package()
