cmake_minimum_required(VERSION 3.5)
project(bitbots_global_planner)

find_package(ament_cmake REQUIRED)
find_package(bitbots_cargo REQUIRED)

cargo_pyo3_extension()

ament_package()
