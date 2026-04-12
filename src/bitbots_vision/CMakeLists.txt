cmake_minimum_required(VERSION 3.8)
project(bitbots_vision)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(soccer_vision_2d_msgs REQUIRED)
find_package(soccer_vision_attribute_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---------- ONNX Runtime ----------
#find_package(onnxruntime CONFIG QUIET)
if(NOT onnxruntime_FOUND)
  message(STATUS "onnxruntime CMake config not found, searching manually...")
  # Search for the directory that contains onnxruntime/core/session/onnxruntime_cxx_api.h
  # so that #include <onnxruntime/core/session/onnxruntime_cxx_api.h> works correctly and
  # relative includes inside the header chain resolve properly.
  find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime/core/session/onnxruntime_cxx_api.h
    HINTS
      ENV CONDA_PREFIX
      ENV PIXI_PREFIX
    PATH_SUFFIXES include
  )
  find_library(ONNXRUNTIME_LIBRARY
    NAMES onnxruntime
    HINTS
      ENV CONDA_PREFIX
      ENV PIXI_PREFIX
    PATH_SUFFIXES lib lib64
  )
  if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
    message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIBRARY}")
    add_library(onnxruntime::onnxruntime IMPORTED SHARED)
    set_target_properties(onnxruntime::onnxruntime PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
      IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
    )
  else()
    message(FATAL_ERROR
      "Could not find ONNX Runtime.\n"
      "  Include dir: ${ONNXRUNTIME_INCLUDE_DIR}\n"
      "  Library:     ${ONNXRUNTIME_LIBRARY}\n"
      "Install onnxruntime or make sure CONDA_PREFIX / PIXI_PREFIX is set correctly."
    )
  endif()
endif()
# ----------------------------------

generate_parameter_library(
  vision_parameters
  config/vision_parameters.yaml
)

# ---------------------------------------------------------------------------
# Processing library (no ONNX dependency) – used by tests and the handler
# ---------------------------------------------------------------------------
add_library(bitbots_vision_processing SHARED
  src/yoeo_processing.cpp
  src/debug_image.cpp
  src/model_config.cpp
)

target_include_directories(bitbots_vision_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(bitbots_vision_processing PUBLIC
  yaml-cpp
  ${OpenCV_LIBS}
)

# ---------------------------------------------------------------------------
# Full vision library (includes ONNX handler) – used by the node and handler tests
# ---------------------------------------------------------------------------
add_library(bitbots_vision_lib SHARED
  src/yoeo_handler.cpp
)

target_include_directories(bitbots_vision_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(bitbots_vision_lib PUBLIC
  bitbots_vision_processing
  vision_parameters
  onnxruntime::onnxruntime
)

ament_target_dependencies(bitbots_vision_lib PUBLIC
  rclcpp
)

# ---------------------------------------------------------------------------
# vision node executable
# ---------------------------------------------------------------------------
add_executable(vision src/vision_node.cpp)

target_link_libraries(vision
  bitbots_vision_lib
)

ament_target_dependencies(vision
  backward_ros
  cv_bridge
  rclcpp
  sensor_msgs
  soccer_vision_2d_msgs
  soccer_vision_attribute_msgs
  std_msgs
  vision_msgs
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_candidate test/test_candidate.cpp)
  target_link_libraries(test_candidate bitbots_vision_processing)

  ament_add_gtest(test_model_config test/test_model_config.cpp)
  target_link_libraries(test_model_config bitbots_vision_processing)

  ament_add_gtest(test_debug_image test/test_debug_image.cpp)
  target_link_libraries(test_debug_image bitbots_vision_processing)

  ament_add_gtest(test_processing test/test_processing.cpp)
  target_link_libraries(test_processing bitbots_vision_processing)

  ament_add_gtest(test_yoeo_handler test/test_yoeo_handler.cpp)
  target_link_libraries(test_yoeo_handler bitbots_vision_lib)
endif()

install(TARGETS vision bitbots_vision_lib bitbots_vision_processing DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
