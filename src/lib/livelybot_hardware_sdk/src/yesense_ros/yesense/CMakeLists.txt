cmake_minimum_required(VERSION 3.8)
project(yesense_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YesenseIMUSetting.msg"
  "msg/YesenseImuEulerAngle.msg"
  "msg/YesenseImuLocation.msg"
  "msg/YesenseImuQuaternion.msg"
  "msg/YesenseImuStatus.msg"
  "msg/YesenseImuUtcTime.msg"
  "msg/YesenseImuMasterGnssData.msg"
  "msg/YesenseImuSlaveGnssData.msg"
  "msg/YesenseImuGnssData.msg"
  "msg/YesenseImuGpsData.msg"
  "msg/YesenseImuSensorData.msg"
  "msg/YesenseImuAllData.msg"
  "msg/YesenseImuCmdResp.msg"
  "msg/YesenseImuVelocity.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

generate_parameter_library(yesense_parameters
  config/yesense_params.yml
)

add_executable(yesense_imu_node
  src/yesense_driver.cpp
  src/yesense_node.cpp
  src/analysis_data.cpp
)

target_include_directories(yesense_imu_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(yesense_imu_node
  rclcpp
  serial
  tf2_ros
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(yesense_imu_node "${cpp_typesupport_target}" yesense_parameters)

install(TARGETS yesense_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
