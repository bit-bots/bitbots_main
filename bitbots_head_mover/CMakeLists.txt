cmake_minimum_required(VERSION 3.8)
project(bitbots_head_mover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(bio_ik REQUIRED)
find_package(bio_ik_msgs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(irobot_events_executor REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_ros REQUIRED)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
generate_parameter_library(
  head_parameters # cmake target name for the parameter library
  config/head_config.yml)

add_executable(move_head src/move_head.cpp)
target_link_libraries(move_head 
  rclcpp::rclcpp
  head_parameters
)


ament_target_dependencies(
  move_head 
#head_capsule
backward_ros
bio_ik
bio_ik_msgs
bitbots_msgs
generate_parameter_library
humanoid_league_msgs
irobot_events_executor
moveit_core
moveit_msgs
moveit_ros_planning
moveit_ros_planning_interface
rclcpp
sensor_msgs
std_msgs
tf2_geometry_msgs
tf2_kdl
tf2_ros


)

install(TARGETS
  move_head
  #head_capsule
  DESTINATION lib/${PROJECT_NAME}) 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})
ament_package()
