cmake_minimum_required(VERSION 2.8.3)
project(white_balancer)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  rosconsole
  message_generation
  cv_bridge
  bitbots_docs
  pluginlib
  nodelet
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/WhiteBalancer.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(white_balancer src/white_balancer.cpp)

add_dependencies(white_balancer ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(white_balancer
    ${catkin_LIBRARIES}
)

# Nodelet library
add_library(test_nodelet src/white_balancer.cpp)
add_dependencies(test_nodelet ${PROJECT_NAME})
target_link_libraries(test_nodelet ${catkin_LIBRARIES})

enable_bitbots_docs()
