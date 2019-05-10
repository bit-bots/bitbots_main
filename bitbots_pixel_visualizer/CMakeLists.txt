cmake_minimum_required(VERSION 2.8.3)
project(bitbots_pixel_visualizer)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  humanoid_league_msgs
  sensor_msgs
  dynamic_reconfigure
  message_generation
)

generate_dynamic_reconfigure_options(
    cfg/PixelVisualizer.cfg
)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs
  humanoid_league_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# add_library(libPF_lib src/CRandomNumberGenerator.cpp include/libPF)
# add_dependencies(libPF_lib ${catkin_EXPORTED_TARGETS})
# target_link_libraries(libPF_lib ${catkin_LIBRARIES})

# add_library(${PROJECT_NAME}_lib src/image_transformer.cpp)
# add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

# add_executable(image_transformer src/image_transformer.cpp)
# target_link_libraries(image_transformer ${catkin_LIBRARIES})
# add_dependencies(image_transformer ${PROJECT_NAME}_gencfg)
# add_dependencies(image_transformer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pixel_visualizer src/pixel_visualizer.cpp)
target_link_libraries(pixel_visualizer ${catkin_LIBRARIES})
add_dependencies(pixel_visualizer ${PROJECT_NAME}_gencfg)
add_dependencies(pixel_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
