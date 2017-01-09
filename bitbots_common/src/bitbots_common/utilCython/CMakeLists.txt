set_source_files_properties(
    joints.pyx png.pyx pydatavector.pyx kalman.pyx kinematicutil.pyx
    PROPERTIES CYTHON_IS_CXX TRUE
)

include_directories(.)
include_directories(../util)
include_directories(../pose) # weil kinimatikutil doof ist
include_directories(../robot) # weil kinimatikutil doof ist
include_directories(${Eigen_INCLUDE_DIRS})

cython_add_module(joints joints.pyx)
cython_add_module(kalman kalman.pyx)
cython_add_module(pydatavectora pydatavector.pyx)

cython_add_module(kinematicutil kinematicutil.pyx)
#cython_add_module(png png.pyx)

set(INSTALL_TARGETS joints kalman pydatavectora kinematicutil)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

#add_library(bitbots_png STATIC png.cpp)
add_library(kinematic_util STATIC kinematic_util.cpp)

#target_link_libraries(bitbots_png ${PNG_LIBRARY})

#target_link_libraries(png bitbots_png debug-common)

target_link_libraries(kinematic_util robot)
target_link_libraries(kinematicutil kinematic_util)


# TODO: das geht nicht weil es dan zwei bitbots_common/util gibt (eins mit .so und eines mit .py)
set_target_properties(${INSTALL_TARGETS}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/utilCython)


#install(
#    TARGETS ${INSTALL_TARGETS}
#    LIBRARY DESTINATION ${PYLIB_INSTALL_PATH}/bitbots_common/util
#)