cmake_minimum_required(VERSION 3.5)
project(bitbots_team_communication)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(Protobuf REQUIRED)

add_custom_target(${PROJECT_NAME}_generate_proto ALL)

add_custom_command(
  TARGET ${PROJECT_NAME}_generate_proto
  COMMAND protoc
  --pyi_out ${CMAKE_SOURCE_DIR}/bitbots_team_communication/
  --python_out ${CMAKE_SOURCE_DIR}/bitbots_team_communication/
  --proto_path ${CMAKE_SOURCE_DIR}/bitbots_team_communication/RobocupProtocol
  ${CMAKE_SOURCE_DIR}/bitbots_team_communication/RobocupProtocol/robocup_extension.proto
  COMMENT "Generating protobuf python interface including stubs")

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


  # The following logic for manually filtering out the files can be removed if 
  # https://github.com/ament/ament_lint/pull/516 is merged and released in our ROS distro
  
  # Collect all .py and .pyi files
  file(GLOB_RECURSE PY_FILES "${CMAKE_CURRENT_LIST_DIR}/**/*.py")
  file(GLOB_RECURSE PYI_FILES "${CMAKE_CURRENT_LIST_DIR}/**/*.pyi")

  # Filter out .py files if a corresponding .pyi file exists
  foreach(py_file IN LISTS PY_FILES)
      # Get the directory and basename (without extension)
      get_filename_component(py_dir "${py_file}" DIRECTORY)
      get_filename_component(py_basename "${py_file}" NAME_WE)

      # Construct the corresponding .pyi file path
      set(pyi_file "${py_dir}/${py_basename}.pyi")
      
      # If a corresponding .pyi file exists, remove the .py file from the list
      if(EXISTS "${pyi_file}")
          list(REMOVE_ITEM PY_FILES "${py_file}")
      endif()
  endforeach()

  ament_mypy(
    CONFIG_FILE "${CMAKE_CURRENT_LIST_DIR}/mypy.ini"
    ${PY_FILES} ${PYI_FILES}
    )
endif()

ament_package()
