cmake_minimum_required(VERSION 3.8)
project(bitbots_head_mover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(move_head src/move_head.cpp)
#add_executable(head_capsule src/head_capsule.cpp)
ament_target_dependencies(
  move_head 
#head_capsule
rclcpp 
bitbots_msgs 
tf2_ros
humanoid_league_msgs
sensor_msgs
moveit_core
moveit_msgs
moveit_ros_planning_interface
moveit_ros_planning
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

ament_package()
