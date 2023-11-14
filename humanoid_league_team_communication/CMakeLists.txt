cmake_minimum_required(VERSION 3.5)
project(humanoid_league_team_communication)

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

protobuf_generate_python(PROTO_PY
  humanoid_league_team_communication/RobocupProtocol/robocup_extension.proto)
add_custom_target(humanoid_league_team_communication ALL DEPENDS ${PROTO_PY})
add_custom_command(TARGET humanoid_league_team_communication
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_PY}
  ${CMAKE_SOURCE_DIR}/humanoid_league_team_communication)

enable_bitbots_docs()
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts/ USE_SOURCE_PERMISSIONS
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Find all python test files
  file(GLOB_RECURSE _pytest_tests test/**/test_*.py)
  # Add each test file to the ament test targets
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV AMENT_PREFIX_INDEX=${ament_index_build_path} 
      PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    )
  endforeach()
endif()
  

ament_package()
