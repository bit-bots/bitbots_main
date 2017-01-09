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

set_source_files_properties(
    walking.pyx zmpwalking.pyx
    PROPERTIES CYTHON_IS_CXX TRUE
)

cython_add_module(zmpwalking zmpwalking.pyx zmp_walk.cpp zmp_walk_parameter.cpp
                    zmp_team_darwin_kinematics.cpp)

set_target_properties(zmpwalking PROPERTIES COMPILE_FLAGS -fPIC
                        LINK_FLAGS -fPIC)

set(INSTALL_TARGETS ${INSTALL_TARGETS} zmpwalking)

target_link_libraries(zmpwalking debug-common utility)

set_target_properties(zmpwalking
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/walking)


# Die kompilierten Module installieren
install(
    TARGETS ${INSTALL_TARGETS}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/bitbots_walking/
)

add_subdirectory(NUBots/utility)

#if(INVERSE_DARWIN_KINEMATICS)
#add_subdirectory(DarwinWalking)
#else()
#add_subdirectory(MinibotWalking)
#endif(INVERSE_DARWIN_KINEMATICS)

