cmake_minimum_required(VERSION 2.8.3)
project(bitbots_recordui_rqt)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED bitbots_docs)
catkin_package()
catkin_python_setup()

enable_bitbots_docs()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
