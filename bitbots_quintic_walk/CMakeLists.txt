cmake_minimum_required(VERSION 3.5)
project(bitbots_quintic_walk)

# Add support for C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

#find_package(bitbots_test REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PythonLibs)
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs COMPONENTS Interpreter Development)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rotconv REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bitbots_splines REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_toolbox REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/WalkDebug.msg"
        "msg/WalkEngineDebug.msg"
        DEPENDENCIES
        std_msgs
        geometry_msgs)

include_directories(include ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
add_compile_options(-Wall -Werror -Wno-unused)

set(SOURCES src/walk_visualizer.cpp
        src/walk_engine.cpp
        src/walk_stabilizer.cpp
        src/walk_ik.cpp
        src/walk_node.cpp)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

set(CODE_LIBRARIES urdfdom_world urdfdom_model)

add_executable(WalkNode ${SOURCES})

ament_target_dependencies(WalkNode
        bitbots_splines
        tf2_eigen
        tf2
        tf2_geometry_msgs
        nav_msgs
        std_msgs
        rotconv
        geometry_msgs
        sensor_msgs
        tf2_ros
        ament_cmake
        rclcpp
        humanoid_league_msgs
        bitbots_msgs
        moveit_ros
        moveit_ros_planning_interface
        control_toolbox)

rosidl_target_interfaces(WalkNode
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(WalkNode ${CODE_LIBRARIES})


# create the lib
#add_library(py_quintic_walk SHARED src/walk_pywrapper.cpp ${SOURCES})
#
#ament_target_dependencies(py_quintic_walk
#        bitbots_splines
#        tf2_eigen
#        tf2
#        tf2_geometry_msgs
#        nav_msgs
#        std_msgs
#        rotconv
#        geometry_msgs
#        sensor_msgs
#        tf2_ros
#        ament_cmake
#        rclcpp
#        humanoid_league_msgs
#        bitbots_msgs
#        moveit_ros
#        moveit_ros_planning_interface)
#
#install(TARGETS py_quintic_walk
#        DESTINATION lib/${PROJECT_NAME})
#
#rosidl_target_interfaces(py_quintic_walk
#        ${PROJECT_NAME} "rosidl_typesupport_cpp")
#
## link
#target_link_libraries(py_quintic_walk ${Boost_LIBRARIES} ${CODE_LIBRARIES})


#if(BUILD_TESTING)
#  enable_bitbots_tests()
#endif()

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

#ament_export_dependencies(bitbots_test)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(rotconv)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(moveit_ros_planning_interface)
ament_export_include_directories(include)
ament_export_libraries(py_quintic_walk ${LIBS})

install(TARGETS WalkNode
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
