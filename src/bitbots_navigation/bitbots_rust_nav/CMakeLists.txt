cmake_minimum_required(VERSION 3.5)
project(bitbots_rust_nav)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

execute_process(
  COMMAND cargo metadata --no-deps --format-version=1
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE CARGO_METADATA_JSON
  OUTPUT_STRIP_TRAILING_WHITESPACE)

string(JSON CARGO_TARGET_DIR GET "${CARGO_METADATA_JSON}" target_directory)

# Python extension suffix, e.g. .cpython-312-x86_64-linux-gnu.so
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c
    "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))"
  OUTPUT_VARIABLE PYTHON_EXT_SUFFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE)

set(PYTHON_INSTALL_DIR
    lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages)

add_custom_target(cargo_rust_nav ALL
    COMMAND ${CMAKE_COMMAND} "-E" "env"
        "PYO3_PYTHON=${Python3_EXECUTABLE}"
        cargo build --profile colcon --package ${PROJECT_NAME}
            --features pyo3/extension-module --quiet
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    USES_TERMINAL
    VERBATIM
)

install(CODE "
    file(COPY
        \"${CARGO_TARGET_DIR}/colcon/lib${PROJECT_NAME}.so\"
        DESTINATION \"\${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}/\")
    file(RENAME
        \"\${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}/lib${PROJECT_NAME}.so\"
        \"\${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}/${PROJECT_NAME}${PYTHON_EXT_SUFFIX}\")
")

ament_package()
