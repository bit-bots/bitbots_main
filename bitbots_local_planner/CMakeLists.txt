cmake_minimum_required(VERSION 3.5)
project(bitbots_local_planner)
add_compile_options(-Wall -Werror -Wno-unused)
set(CMAKE_CXX_STANDARD 17)


find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  angles
  nav2_costmap_2d
  geometry_msgs
  nav2_core
  nav2_util
  nav_2d_utils
  nav_msgs
  pluginlib
  tf2_ros
  tf2_geometry_msgs
)

add_library(bitbots_local_planner SHARED
src/bitbots_local_planner.cpp
src/transform_global_plan.cpp
)

ament_target_dependencies(bitbots_local_planner
  ${dependencies}
)

install(TARGETS bitbots_local_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(bitbots_local_planner)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core blp_plugin.xml)

ament_package()