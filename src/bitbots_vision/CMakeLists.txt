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
# find_package(onnxruntime CONFIG QUIET)
if(NOT onnxruntime_FOUND)
  message(STATUS "onnxruntime CMake config not found, searching manually...")
  # Search for the directory that contains
  # onnxruntime/core/session/onnxruntime_cxx_api.h so that #include
  # <onnxruntime/core/session/onnxruntime_cxx_api.h> works correctly and
  # relative includes inside the header chain resolve properly.
  find_path(
    ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime/core/session/onnxruntime_cxx_api.h
    HINTS ENV CONDA_PREFIX ENV PIXI_PREFIX
    PATH_SUFFIXES include)
  find_library(
    ONNXRUNTIME_LIBRARY
    NAMES onnxruntime
    HINTS ENV CONDA_PREFIX ENV PIXI_PREFIX
    PATH_SUFFIXES lib lib64)
  if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
    message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIBRARY}")
    add_library(onnxruntime::onnxruntime IMPORTED SHARED)
    set_target_properties(
      onnxruntime::onnxruntime
      PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
                 IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}")
  else()
    message(
      FATAL_ERROR
        "Could not find ONNX Runtime.\n"
        "  Include dir: ${ONNXRUNTIME_INCLUDE_DIR}\n"
        "  Library:     ${ONNXRUNTIME_LIBRARY}\n" "Install onnxruntime.")
  endif()
endif()
# ----------------------------------

# ---------- TensorRT (optional; preferred over ONNX Runtime when found) ----------
# TensorRT ships no CMake config package on JetPack, so this is a manual
# find, mirroring the onnxruntime search above. Not present in the x86 dev
# environment -- if not found here, only the ONNX Runtime backend is built
# and nothing else about the build changes.
find_path(
  TENSORRT_INCLUDE_DIR
  NAMES NvInfer.h
  HINTS ENV CONDA_PREFIX ENV PIXI_PREFIX ENV CUDA_HOME ENV CUDA_PATH
  PATHS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/local/cuda/include /usr/local/cuda-11.4/include
  PATH_SUFFIXES include)
find_library(
  TENSORRT_NVINFER_LIBRARY
  NAMES nvinfer
  HINTS ENV CONDA_PREFIX ENV PIXI_PREFIX
  PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu
  PATH_SUFFIXES lib lib64)
find_library(
  TENSORRT_NVONNXPARSER_LIBRARY
  NAMES nvonnxparser
  HINTS ENV CONDA_PREFIX ENV PIXI_PREFIX
  PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu
  PATH_SUFFIXES lib lib64)
find_path(
  CUDA_INCLUDE_DIR
  NAMES cuda_runtime_api.h
  HINTS ENV CUDA_HOME ENV CUDA_PATH
  PATHS /usr/local/cuda /usr/local/cuda-11.4
  PATH_SUFFIXES include)
find_library(
  CUDA_CUDART_LIBRARY
  NAMES cudart
  HINTS ENV CUDA_HOME ENV CUDA_PATH
  PATHS /usr/local/cuda /usr/local/cuda-11.4 /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu
  PATH_SUFFIXES lib64 lib)

if(TENSORRT_INCLUDE_DIR
   AND TENSORRT_NVINFER_LIBRARY
   AND TENSORRT_NVONNXPARSER_LIBRARY
   AND CUDA_INCLUDE_DIR
   AND CUDA_CUDART_LIBRARY)
  set(BITBOTS_VISION_HAS_TENSORRT TRUE)
  message(STATUS "Found TensorRT: ${TENSORRT_NVINFER_LIBRARY} -- TensorRtHandler will be built and preferred at runtime")
else()
  set(BITBOTS_VISION_HAS_TENSORRT FALSE)
  message(STATUS "TensorRT not found -- only the ONNX Runtime backend will be built")
endif()
# -----------------------------------------------------------------------------

generate_parameter_library(vision_parameters config/vision_parameters.yaml)

# ---------------------------------------------------------------------------
# Processing library (no ONNX dependency) – used by tests and the handler
# ---------------------------------------------------------------------------
add_library(bitbots_vision_processing SHARED
            src/rfdetr_processing.cpp src/debug_image.cpp src/model_config.cpp)

target_include_directories(
  bitbots_vision_processing
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(bitbots_vision_processing PUBLIC yaml-cpp ${OpenCV_LIBS})

# ---------------------------------------------------------------------------
# Full vision library (includes the inference handler(s)) – used by the node
# and handler tests
# ---------------------------------------------------------------------------
set(BITBOTS_VISION_LIB_SOURCES src/rfdetr_handler.cpp)
if(BITBOTS_VISION_HAS_TENSORRT)
  list(APPEND BITBOTS_VISION_LIB_SOURCES src/tensorrt_handler.cpp)
endif()

add_library(bitbots_vision_lib SHARED ${BITBOTS_VISION_LIB_SOURCES})

target_include_directories(
  bitbots_vision_lib
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  bitbots_vision_lib PUBLIC bitbots_vision_processing vision_parameters
                            onnxruntime::onnxruntime)

if(BITBOTS_VISION_HAS_TENSORRT)
  # Deliberately NOT target_include_directories() (-I/-isystem) here: on
  # Jetson, apt-installed TensorRT dumps its headers into
  # /usr/include/<arch>-linux-gnu -- the SAME directory as the system's own
  # glibc headers, which are for a different/older glibc than the one this
  # conda-forge cross-compiler's libstdc++ was built against. -I/-isystem
  # are searched BEFORE the compiler's own bundled sysroot, so adding that
  # directory that way lets the system's mismatched <bits/pthreadtypes.h>
  # (etc.) shadow the toolchain's own matching version, breaking
  # <mutex>/<memory> for every translation unit in this target (e.g.
  # "'__PTHREAD_SPINS' was not declared in this scope" deep inside
  # libstdc++'s <mutex> -- happens even in code that never touches
  # TensorRT). -idirafter is only consulted as a last resort, after the
  # compiler's built-in system directories, so the toolchain's own glibc
  # headers are still found first for anything that exists in both places,
  # and only TensorRT/CUDA-specific headers (found nowhere else) fall
  # through to this directory.
  #
  # Each flag+path is glued into a single token ("-idirafterPATH", no space)
  # rather than passed as two separate list elements ("-idirafter" "PATH").
  # CMake's compile-options accumulation can silently drop a *repeated
  # identical flag string* (a second bare "-idirafter") when flattening the
  # option list across targets, stranding the path that followed it as a
  # bare argument -- GCC then warns it's an "unused linker input file" and
  # the include never takes effect. GCC accepts both "-idirafter dir" and
  # the glued "-idirafterdir" form; gluing avoids the pitfall entirely since
  # there's no repeated flag string left to collide on.
  target_compile_options(bitbots_vision_lib PUBLIC
                          "-idirafter${TENSORRT_INCLUDE_DIR}" "-idirafter${CUDA_INCLUDE_DIR}")
  target_link_libraries(bitbots_vision_lib PUBLIC ${TENSORRT_NVINFER_LIBRARY}
                                                  ${TENSORRT_NVONNXPARSER_LIBRARY} ${CUDA_CUDART_LIBRARY})
  # PUBLIC so vision_node.cpp (compiled into the `vision` executable, which
  # links this library) also sees this define and prefers TensorRtHandler.
  target_compile_definitions(bitbots_vision_lib PUBLIC BITBOTS_VISION_HAS_TENSORRT)
endif()

ament_target_dependencies(bitbots_vision_lib PUBLIC rclcpp)

# ---------------------------------------------------------------------------
# vision node executable
# ---------------------------------------------------------------------------
add_executable(vision src/vision_node.cpp)

target_link_libraries(vision bitbots_vision_lib)

ament_target_dependencies(
  vision
  backward_ros
  cv_bridge
  rclcpp
  sensor_msgs
  soccer_vision_2d_msgs
  soccer_vision_attribute_msgs
  std_msgs
  vision_msgs)

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

  ament_add_gtest(test_rfdetr_processing test/test_rfdetr_processing.cpp)
  target_link_libraries(test_rfdetr_processing bitbots_vision_processing)

  ament_add_gtest(test_rfdetr_handler test/test_rfdetr_handler.cpp)
  target_link_libraries(test_rfdetr_handler bitbots_vision_lib)
endif()

install(TARGETS vision bitbots_vision_lib bitbots_vision_processing
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
