cmake_minimum_required(VERSION 3.5)
project(bitbots_quintic_walk)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(bitbots_test REQUIRED)
find_package(ament_cmake REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/WalkDebug.msg"
  "msg/WalkEngineDebug.msg"
  DEPENDENCIES builtin_interfaces)

set(INCLUDE_DIRS include ${bitbots_test_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${bitbots_test_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${bitbots_test_LIBRARIES} ${ament_cmake_LIBRARIES})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
add_compile_options(-Wall -Werror -Wno-unused)

set(SOURCES src/walk_visualizer.cpp src/walk_engine.cpp src/walk_stabilizer.cpp
  src/walk_ik.cpp src/walk_node.cpp)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(PythonLibs)
if(PYTHONLIBS_VERSION_STRING VERSION_LESS "3.8")
  find_package(Boost REQUIRED COMPONENTS python3)
else()
  find_package(Boost REQUIRED COMPONENTS python)
endif()
find_package(PythonLibs COMPONENTS Interpreter Development)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

enable_bitbots_docs()

generate_dynamic_reconfigure_options(cfg/bitbots_quintic_walk_params.cfg
  cfg/bitbots_quintic_walk_engine_params.cfg)

set(CODE_LIBRARIES urdfdom_world urdfdom_model)

add_executable(WalkNode ${SOURCES})

target_link_libraries(WalkNode ${LIBS} ${CODE_LIBRARIES})

# create the lib
add_library(py_quintic_walk SHARED src/walk_pywrapper.cpp ${SOURCES})

# link
target_link_libraries(py_quintic_walk ${LIBS} ${Boost_LIBRARIES}
  ${CODE_LIBRARIES})

set_target_properties(py_quintic_walk
  PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY
  ${CATKIN_PACKAGE_PYTHON_DESTINATION})

if(BUILD_TESTING)
  enable_bitbots_tests()
endif()

ament_export_dependencies(bitbots_test)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(py_quintic_walk ${LIBS})

ament_package()
