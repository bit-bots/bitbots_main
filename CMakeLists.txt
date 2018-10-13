cmake_minimum_required(VERSION 2.8.3)
project(bitbots_world_model)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  humanoid_league_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/world_model.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(libPF_lib src/CRandomNumberGenerator.cpp include/libPF)
add_dependencies(libPF_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(libPF_lib ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_lib src/ObstacleStates.cpp include/${PROJECT_NAME}/ObstacleStates.h src/ObservationModels.cpp include/${PROJECT_NAME}/ObservationModels.h src/MovementModels.cpp include/${PROJECT_NAME}/MovementModels.h src/StateDistributions.cpp include/${PROJECT_NAME}/StateDistributions.h)
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

add_executable(world_model src/world_model.cpp)
target_link_libraries(world_model ${catkin_LIBRARIES} libPF_lib ${PROJECT_NAME}_lib)
add_dependencies(world_model ${PROJECT_NAME}_gencfg)
add_dependencies(world_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
