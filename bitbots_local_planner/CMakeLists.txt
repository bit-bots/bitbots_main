cmake_minimum_required(VERSION 2.8.3)
project(bitbots_local_planner)
add_compile_options(-Wall -Werror -Wno-unused)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
  COMPONENTS
  angles
  cmake_modules
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/BBPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bitbots_local_planner
  CATKIN_DEPENDS dynamic_reconfigure pluginlib roscpp
)

add_library(bitbots_local_planner
  src/bitbots_local_planner.cpp
  src/transform_global_plan.cpp
)

target_link_libraries(bitbots_local_planner ${catkin_LIBRARIES})
add_dependencies(bitbots_local_planner bitbots_local_planner_gencfg)
