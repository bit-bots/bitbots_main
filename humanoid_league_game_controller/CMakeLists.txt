cmake_minimum_required(VERSION 3.5)
project(humanoid_league_game_controller)

find_package(ament_cmake_python REQUIRED)
find_package(bitbots_docs REQIRED)

enable_bitbots_docs()
ament_python_install_package(${PROJECT_NAME})
ament_package()
