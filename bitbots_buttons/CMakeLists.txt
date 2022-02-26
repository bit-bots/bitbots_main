cmake_minimum_required(VERSION 3.5)
project(bitbots_buttons)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(INCLUDE_DIRS ${std_msgs_INCLUDE_DIRS}${ament_cmake_INCLUDE_DIRS}
  ${bitbots_docs_INCLUDE_DIRS} ${bitbots_msgs_INCLUDE_DIRS}
  ${humanoid_league_msgs_INCLUDE_DIRS})

include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${std_msgs_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS}
  ${bitbots_docs_LIBRARY_DIRS} ${bitbots_msgs_LIBRARY_DIRS}
  ${humanoid_league_msgs_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${std_msgs_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${bitbots_docs_LIBRARIES} ${bitbots_msgs_LIBRARIES} ${humanoid_league_msgs_LIBRARIES})

#enable_bitbots_docs()

install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/button_node.py DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(std_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(bitbots_msgs)
ament_export_dependencies(humanoid_league_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
