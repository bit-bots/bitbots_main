set_source_files_properties(
    pypose.pyx
    PROPERTIES CYTHON_IS_CXX TRUE
)

include_directories(.)

cython_add_module(pypose pypose.pyx)

set(INSTALL_TARGETS pypose)

set_target_properties(${INSTALL_TARGETS}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/pose)


install(
    TARGETS ${INSTALL_TARGETS}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/bitbots_common/pose
)
