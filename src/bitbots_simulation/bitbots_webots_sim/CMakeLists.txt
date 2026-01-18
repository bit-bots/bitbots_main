cmake_minimum_required(VERSION 3.5)
project(bitbots_webots_sim)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)

enable_bitbots_docs()

install(DIRECTORY protos DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/fix_urdf_for_webots.py
           scripts/imu_lut_gen.py
           scripts/localization_faker.py
           scripts/start_simulator.py
           scripts/start_single.py
           scripts/start_webots_ros_supervisor.py
           scripts/field_publisher.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_mypy REQUIRED)
  ament_mypy(CONFIG_FILE "${CMAKE_CURRENT_LIST_DIR}/mypy.ini")
endif()

ament_python_install_package(${PROJECT_NAME})

ament_package()
