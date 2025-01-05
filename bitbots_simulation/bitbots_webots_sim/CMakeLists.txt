cmake_minimum_required(VERSION 3.5)
project(bitbots_webots_sim)

find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)

enable_bitbots_docs()

install(DIRECTORY protos DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/fix_urdf_for_webots.py scripts/imu_lut_gen.py
           scripts/localization_faker.py scripts/start_simulator.py
           scripts/start_single.py scripts/start_webots_ros_supervisor.py
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_package()
