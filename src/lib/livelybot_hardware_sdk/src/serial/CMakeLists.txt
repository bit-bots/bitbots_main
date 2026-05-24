cmake_minimum_required(VERSION 3.8)
project(serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

set(serial_SRCS
  src/serial.cc
  src/impl/unix.cc
  src/impl/list_ports/list_ports_linux.cc
)

add_library(serial SHARED ${serial_SRCS})
target_include_directories(serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(serial rt pthread)
target_compile_features(serial PUBLIC cxx_std_14)

install(TARGETS serial
  EXPORT serialTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT serialTargets
  FILE serialTargets.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(serialTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(serial)

ament_package()
