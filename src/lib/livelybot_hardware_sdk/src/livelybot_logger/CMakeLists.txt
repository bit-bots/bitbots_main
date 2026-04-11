cmake_minimum_required(VERSION 3.8)
project(livelybot_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(livelybot_power REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LoggerStatus.msg"
  "msg/LoggerOperation.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Logger interface library (used by other nodes to publish operations)
add_library(livelybot_logger_iface SHARED src/logger_interface.cpp)
ament_target_dependencies(livelybot_logger_iface rclcpp)
target_include_directories(livelybot_logger_iface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(livelybot_logger_iface "${cpp_typesupport_target}")

generate_parameter_library(logger_parameters
  config/logger_params.yml
)

# Logger node executable
add_executable(${PROJECT_NAME}_node src/logger_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs sensor_msgs tf2 livelybot_power)
target_include_directories(${PROJECT_NAME}_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
rosidl_get_typesupport_target(cpp_typesupport_target_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node livelybot_logger_iface logger_parameters "${cpp_typesupport_target_node}")

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS livelybot_logger_iface
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
