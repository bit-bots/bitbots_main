cmake_minimum_required(VERSION 3.5)
project(bitbots_pybullet_sim)

find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)

enable_bitbots_docs()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/simulation_headless.py scripts/simulation_with_gui.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
