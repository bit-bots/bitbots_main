cmake_minimum_required(VERSION 3.5)
project(bitbots_parameter_blackboard)

find_package(bitbots_docs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

enable_bitbots_docs()

# Python install
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/parameter_blackboard DESTINATION lib/${PROJECT_NAME})

# Copy config and launch files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Find all python test files
  file(GLOB_RECURSE pytest_tests_ test/test_*.py)
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
endif()

ament_package()
