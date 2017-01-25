include_directories(../../../../bitbots_misc/bitbots_common/src)
include_directories(../../../../bitbots_misc/bitbots_common/src/bitbots_common/pose)
include_directories(../../../../bitbots_misc/bitbots_common/src/bitbots_common/util)
include_directories(../../../../bitbots_misc/bitbots_common/src/bitbots_common/utilCython)
include_directories(../../../../bitbots_misc/bitbots_common/src/bitbots_common/robot)
include_directories(.)
#include_directories(..)
include_directories(lowlevel)
include_directories(lowlevel/controller)
include_directories(${Eigen_INCLUDE_DIRS})

add_subdirectory(lowlevel)


set_source_files_properties(
    cm730_node.pyx cm730.pyx
    PROPERTIES CYTHON_IS_CXX TRUE
)

#cython_add_module(cm730_node cm730_node.pyx)
cython_add_module(cm730 cm730.pyx)

set_target_properties(cm730
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

# Die kompilierten Module installieren
install(
    TARGETS ${INSTALL_TARGETS}
    #LIBRARY DESTINATION ${PYLIB_INSTALL_PATH}/bitbots/motion
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/bitbots_cm730/
)

