cmake_minimum_required(VERSION 2.8.3)
project(white_balancer)


set(CMAKE_BUILD_TYPE Debug)

FIND_PACKAGE (OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  rosconsole
  message_generation
  cv_bridge
)

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/white_balancer.cfg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(white_balancer src/white_balancer.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(white_balancer
    ${catkin_LIBRARIES}
)
