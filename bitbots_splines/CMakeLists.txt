cmake_minimum_required(VERSION 3.5)
project(bitbots_splines)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(bio_ik REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS}
  ${bitbots_docs_INCLUDE_DIRS} ${bio_ik_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS} ${bitbots_docs_LIBRARY_DIRS} ${bio_ik_LIBRARY_DIRS}
  ${rclcpp_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES} ${bitbots_docs_LIBRARIES} ${bio_ik_LIBRARIES}
  ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
add_compile_options(-Wall -Werror -Wno-unused)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

enable_bitbots_docs()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(SOURCES src/Spline/polynom.cpp src/Spline/smooth_spline.cpp
  src/Spline/spline.cpp src/Spline/pose_spline.cpp src/Spline/position_spline.cpp
  src/Utils/newton_binomial.cpp src/Utils/combination.cpp
  include/bitbots_splines/abstract_engine.h
  include/bitbots_splines/abstract_stabilizer.h
  include/bitbots_splines/abstract_ik.h
  include/bitbots_splines/abstract_visualizer.h
  include/bitbots_splines/reference_goals.h
  include/bitbots_splines/dynamic_balancing_goal.h)

set(CODE_LIBRARIES)

add_library(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  LIBRARY
  DESTINATION lib)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(bio_ik)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS})

ament_package()
