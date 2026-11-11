cmake_minimum_required(VERSION 3.5)
project(bitbots_bringup)

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(bitbots_cargo REQUIRED)

enable_bitbots_docs()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

cargo_binary(bitbots_compose_server)
cargo_binary(bitbots_compose_tui)
cargo_test(bitbots_compose_file)

ament_package()
