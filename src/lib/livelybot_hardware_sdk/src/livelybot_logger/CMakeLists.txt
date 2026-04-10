cmake_minimum_required(VERSION 3.0.2)
project(livelybot_logger)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  livelybot_power
  livelybot_serial
  livelybot_msg
)

add_message_files(
  FILES
  LoggerStatus.msg
  LoggerOperation.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES livelybot_logger
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime livelybot_power livelybot_serial livelybot_msg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/logger_node.cpp src/logger_interface.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} livelybot_logger_gencpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#.so
add_library(${PROJECT_NAME} SHARED src/logger_interface.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} livelybot_logger_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

#add_dependencies(livelybot_serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

#for intall

install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装 launch 文件
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)

install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
# 安装头文件到 install/include
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})