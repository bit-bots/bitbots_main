cmake_minimum_required(VERSION 3.5)
project(ros2_numpy)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(rclpy REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(pointclouds test/test_pointclouds.py)
  ament_add_pytest_test(images test/test_images.py)
  ament_add_pytest_test(occupancygrids test/test_occupancygrids.py)
  ament_add_pytest_test(geometry test/test_geometry.py)
  ament_add_pytest_test(quaternions test/test_quat.py)
  ament_add_pytest_test(laserscan test/test_laserscan.py)
endif()

##############
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_package()
