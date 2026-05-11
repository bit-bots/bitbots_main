# Build test_base_footprint
ament_add_gtest(test_base_footprint
    test_base_footprint.cpp)

find_package(sensor_msgs REQUIRED)

ament_target_dependencies(test_base_footprint
  sensor_msgs
)

target_link_libraries(test_base_footprint
  base_footprint_node
)
