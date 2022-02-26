cmake_minimum_required(VERSION 3.5)
project(bitbots_dynup)

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
find_package(moveit_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
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

find_package(ros2_python_extension REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/DynupEngineDebug.msg"
        "msg/DynupPoses.msg"
        DEPENDENCIES
        std_msgs
        geometry_msgs)

include_directories(include ${PYTHON_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
add_compile_options(-Wall -Werror -Wno-unused -fPIC)

set(SOURCES src/dynup_engine.cpp
        src/dynup_ik.cpp
        src/dynup_node.cpp
        src/dynup_pywrapper.cpp
        src/dynup_stabilizer.cpp)


set(CODE_LIBRARIES urdfdom_world urdfdom_model)

add_executable(DynupNode ${SOURCES})

ament_target_dependencies(DynupNode
        ament_cmake
        bitbots_msgs
        bitbots_splines
        control_msgs
        control_toolbox
        geometry_msgs
        moveit_ros
        moveit_ros_planning_interface
        rclcpp
        rotconv
        sensor_msgs
        std_msgs
        tf2
        tf2_eigen
        tf2_geometry_msgs
        tf2_ros
        Eigen3)

rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(DynupNode ${CODE_LIBRARIES} "${cpp_typesupport_target}")


# create the lib
pybind11_add_module(libpy_dynup SHARED
        src/dynup_pywrapper.cpp ${SOURCES})
ament_target_dependencies(libpy_dynup PUBLIC
        ament_cmake
        bitbots_msgs
        bitbots_splines
        control_msgs
        control_toolbox
        geometry_msgs
        moveit_ros
        moveit_ros_planning_interface
        rclcpp
        ros2_python_extension
        rotconv
        sensor_msgs
        std_msgs
        tf2
        tf2_eigen
        tf2_geometry_msgs
        tf2_ros)

target_link_libraries(libpy_dynup PRIVATE "${cpp_typesupport_target}")

ament_python_install_package(bitbots_dynup_py)

install(TARGETS libpy_dynup
        DESTINATION "${PYTHON_INSTALL_DIR}/bitbots_dynup_py"
        )

#if(BUILD_TESTING)
#  enable_bitbots_tests()
#endif()

include(${CMAKE_BINARY_DIR}/../bitbots_docs/enable_bitbots_docs.cmake)
enable_bitbots_docs()

install(TARGETS DynupNode
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})


ament_package()
