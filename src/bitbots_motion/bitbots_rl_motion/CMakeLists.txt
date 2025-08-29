cmake_minimum_required(VERSION 3.5)
project(bitbots_rl_motion)

find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)

enable_bitbots_docs()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rl_walk_models DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/rl_walk.py DESTINATION lib/${PROJECT_NAME})

ament_package()
