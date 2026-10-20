cmake_minimum_required(VERSION 3.21)
project(bitbots_cargo)

find_package(ament_cmake REQUIRED)

ament_package(CONFIG_EXTRAS cmake/bitbots_cargo.cmake.in)
