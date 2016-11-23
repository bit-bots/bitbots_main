include_directories(../../../bitbots_common/src)
include_directories(../../../bitbots_common/src/bitbots_common/pose)
include_directories(../../../bitbots_common/src/bitbots_common/util)
include_directories(../../../bitbots_common/src/bitbots_common/utilCython)
include_directories(../../../bitbots_common/src/bitbots_common/debug)
include_directories(../../../bitbots_common/src/bitbots_common/robot)
include_directories(../../../bitbots_ipc/src)
include_directories(../../../bitbots_ipc/src/bitbots_ipc)
include_directories(.)
#include_directories(..)
include_directories(${Eigen_INCLUDE_DIRS})

add_subdirectory(lowlevel)
add_subdirectory(walking)


set_source_files_properties(
    basemotionserver.pyx motionserver.pyx simulationmotionserver.pyx
    animation.pyx standuphandler.pyx
    PROPERTIES CYTHON_IS_CXX TRUE
)

cython_add_module(basemotionserver basemotionserver.pyx)
cython_add_module(motionserver motionserver.pyx)
cython_add_module(simulationmotionserver simulationmotionserver.pyx)
cython_add_module(animation animation.pyx)
cython_add_module(standuphandler standuphandler.pyx)

set(INSTALL_TARGETS ${INSTALL_TARGETS} accmovementtracker)

set(INSTALL_TARGETS animation standuphandler basemotionserver motionserver simulationmotionserver )

target_link_libraries(standuphandler debug-common)
target_link_libraries(animation debug-common)
target_link_libraries(basemotionserver debug-common)
target_link_libraries(motionserver debug-common)
target_link_libraries(simulationmotionserver debug-common)

set_target_properties(standuphandler animation basemotionserver motionserver simulationmotionserver
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

# Die kompilierten Module installieren
install(
    TARGETS ${INSTALL_TARGETS}
    #LIBRARY DESTINATION ${PYLIB_INSTALL_PATH}/bitbots/motion
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/python2.7/bitbots_motion/
)

