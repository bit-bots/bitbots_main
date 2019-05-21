cmake_minimum_required(VERSION 2.8.3)
project(white_balancer)


set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  rosconsole
  message_generation
  OpenCV
  cv_bridge
)

set (CMAKE_CXX_STANDARD 11)

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
