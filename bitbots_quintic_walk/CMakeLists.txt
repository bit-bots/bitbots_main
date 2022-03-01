cmake_minimum_required(VERSION 3.5)
project(bitbots_quintic_walk)

# Add support for C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

#find_package(bitbots_test REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(bitbots_splines REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rotconv REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(fp REQUIRED)
find_package(range-v3 REQUIRED)

find_package(ros2_python_extension REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/WalkDebug.msg"
        "msg/WalkEngineDebug.msg"
        DEPENDENCIES
        std_msgs
        geometry_msgs
        Python3)

include_directories(include ${PYTHON_INCLUDE_DIRS})

add_compile_options(-Wall -Werror -Wno-unused)

set(SOURCES src/walk_visualizer.cpp
        src/walk_engine.cpp
        src/walk_stabilizer.cpp
        src/walk_ik.cpp
        src/walk_node.cpp)

add_executable(WalkNode ${SOURCES})

ament_target_dependencies(WalkNode
        ament_cmake
        bitbots_msgs
        bitbots_splines
        control_msgs
        control_toolbox
        geometry_msgs
        humanoid_league_msgs
        moveit_ros
        moveit_ros_planning_interface
        nav_msgs
        rclcpp
        rotconv
        sensor_msgs
        std_msgs
        tf2
        tf2_eigen
        tf2_geometry_msgs
        tf2_ros
        Eigen3
        fmt
        fp
        range-v3)

rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(WalkNode "${cpp_typesupport_target}")


# create the lib
pybind11_add_module(libpy_quintic_walk SHARED
        src/walk_pywrapper.cpp ${SOURCES})
ament_target_dependencies(libpy_quintic_walk PUBLIC
        ament_cmake
        bitbots_msgs
        bitbots_splines
        control_msgs
        control_toolbox
        geometry_msgs
        humanoid_league_msgs
        moveit_ros
        moveit_ros_planning_interface
        nav_msgs
        rclcpp
        ros2_python_extension
        rotconv
        sensor_msgs
        std_msgs
        tf2
        tf2_eigen
        tf2_geometry_msgs
        tf2_ros
        fmt
        fp
        range-v3)

target_link_libraries(libpy_quintic_walk PRIVATE "${cpp_typesupport_target}")

ament_python_install_package(bitbots_quintic_walk_py)

install(TARGETS libpy_quintic_walk
        DESTINATION "${PYTHON_INSTALL_DIR}/bitbots_quintic_walk_py"
        )

#if(BUILD_TESTING)
#  enable_bitbots_tests()
#endif()

enable_bitbots_docs()

install(TARGETS WalkNode
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})


ament_package()
