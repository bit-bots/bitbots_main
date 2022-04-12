cmake_minimum_required(VERSION 3.5)
project(bitbots_animation_server)

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

enable_bitbots_docs()

ament_export_dependencies(actionlib)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(humanoid_league_msgs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION lib/${PROJECT_NAME})


ament_package()
