set_source_files_properties(
    mitecom.pyx
    PROPERTIES CYTHON_IS_CXX TRUE
)

include_directories(.)

cython_add_module(mitecom mitecom.pyx mitecom.cpp mitecom-network.cpp
    mitecom-handler.cpp)

set_target_properties(mitecom
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/mitecom)

install(
    TARGETS mitecom
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/bitbots_team_communication/mitecom)
