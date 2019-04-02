cmake_minimum_required(VERSION 2.8.3)
project(bitbots_world_model)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  eigen_conversions
  geometry_msgs
  humanoid_league_msgs
  particle_filter
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)


find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenMP)
if(OPENMP_FOUND)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
if(MSVC90 OR MSVC10)
if(MSVC90)
set(OPENMP_DLL VCOMP90)
elseif(MSVC10)
set(OPENMP_DLL VCOMP100)
endif(MSVC90)
set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DELAYLOAD:${OPENMP_DLL}D.dll")
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DELAYLOAD:${OPENMP_DLL}.dll")
endif(MSVC90 OR MSVC10)

else(OPENMP_FOUND)
message (STATUS "OpenMP not found")
endif()

if (MSVC)
Set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /O2 ${SSE_FLAGS}")
else (MSVC)
set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS} ${SSE_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g ${SSE_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE " ${CMAKE_CXX_FLAGS} -O3 ${SSE_FLAGS}")
endif (MSVC)


generate_dynamic_reconfigure_options(
  cfg/world_model.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib particle_filter
  CATKIN_DEPENDS geometry_msgs particle_filter roscpp std_msgs visualization_msgs humanoid_league_msgs dynamic_reconfigure eigen_conversions

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_lib src/ObstacleStates.cpp include/${PROJECT_NAME}/ObstacleStates.h src/ObservationModels.cpp include/${PROJECT_NAME}/ObservationModels.h src/MovementModels.cpp include/${PROJECT_NAME}/MovementModels.h src/StateDistributions.cpp include/${PROJECT_NAME}/StateDistributions.h include/${PROJECT_NAME}/Resampling.h)
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

add_executable(world_model src/world_model.cpp)
target_link_libraries(world_model ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
add_dependencies(world_model ${PROJECT_NAME}_gencfg)
add_dependencies(world_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
