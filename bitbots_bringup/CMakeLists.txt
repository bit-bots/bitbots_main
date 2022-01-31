cmake_minimum_required(VERSION 3.5)
project(bitbots_bringup)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)

set(INCLUDE_DIRS ${bitbots_docs_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${bitbots_docs_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${bitbots_docs_LIBRARIES} ${ament_cmake_LIBRARIES})

#enable_bitbots_docs()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/dummy_imu.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/motor_goals_viz_helper.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(bitbots_docs)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
