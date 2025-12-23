cmake_minimum_required(VERSION 3.5)
project(bitbots_rust_test)

find_package(ament_cmake REQUIRED)
find_package(bitbots_cargo REQUIRED)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

# put ros package dependencies here.
r2r_cargo(std_msgs               # just to test that it works
          rcl                    # we need the c ros2 api
          rcl_action             # as of r2r 0.1.0, we also need the action api
          rosgraph_msgs          # for the Clock message
          bitbots_msgs           # our custom message types
         )

# install binaries
if(WIN32)
  set(SUFFIX ".exe")
else()
  set(SUFFIX "")
endif()

execute_process(
  COMMAND cargo metadata --no-deps --format-version=1
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE CARGO_METADATA_JSON
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Extract "target_directory" from JSON
string(JSON CARGO_TARGET_DIR
  GET "${CARGO_METADATA_JSON}" target_directory
)

message(STATUS "Cargo target dir: ${CARGO_TARGET_DIR}")

install(PROGRAMS
  ${CARGO_TARGET_DIR}/colcon/${PROJECT_NAME}${SUFFIX}
  DESTINATION lib/${PROJECT_NAME}
)

# we need this for ros/colcon
ament_package()
