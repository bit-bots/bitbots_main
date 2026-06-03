cmake_minimum_required(VERSION 3.5)
project(bitbots_bringup)

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

enable_bitbots_docs()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(WIN32)
  set(SUFFIX ".exe")
else()
  set(SUFFIX "")
endif()

execute_process(
  COMMAND cargo metadata --no-deps --format-version=1
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE CARGO_METADATA_JSON
  OUTPUT_STRIP_TRAILING_WHITESPACE)

string(JSON CARGO_TARGET_DIR GET "${CARGO_METADATA_JSON}" target_directory)

add_custom_target(cargo_teamplayer ALL
    COMMAND ${CMAKE_COMMAND} "-E" "env" "cargo" "build" "--profile" "colcon" "--package" "teamplayer" "--quiet"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    USES_TERMINAL
    VERBATIM
)

install(PROGRAMS ${CARGO_TARGET_DIR}/colcon/teamplayer${SUFFIX}
        DESTINATION lib/${PROJECT_NAME})

ament_package()
