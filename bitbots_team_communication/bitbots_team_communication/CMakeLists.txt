cmake_minimum_required(VERSION 3.5)
project(bitbots_team_communication)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bitbots_utils REQUIRED)
find_package(bitbots_docs REQUIRED)

find_package(Protobuf REQUIRED)

protobuf_generate_python(
  PROTO_PY bitbots_team_communication/RobocupProtocol/robocup_extension.proto)

add_custom_target(
  bitbots_team_communication ALL
  DEPENDS ${PROTO_PY}
  COMMENT "Generating protobuf")
add_custom_command(
  TARGET bitbots_team_communication
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_PY}
          ${CMAKE_SOURCE_DIR}/bitbots_team_communication
  COMMENT "Copying protobuf to source dir")

enable_bitbots_docs()
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts/ USE_SOURCE_PERMISSIONS
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Find all python test files
  file(GLOB_RECURSE pytest_tests_ test/**/test_*.py)
  # Add each test file to the ament test targets
  foreach(test_path_ ${pytest_tests_})
    get_filename_component(test_name_ ${test_path_} NAME_WE)
    ament_add_pytest_test(
      ${test_name_}
      ${test_path_}
      APPEND_ENV
      AMENT_PREFIX_INDEX=${ament_index_build_path}
      PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT
      60
      WORKING_DIRECTORY
      "${CMAKE_SOURCE_DIR}")
  endforeach()

  find_package(ament_cmake_mypy REQUIRED)
  ament_mypy(CONFIG_FILE "${CMAKE_CURRENT_LIST_DIR}/mypy.ini")
endif()

ament_package()
