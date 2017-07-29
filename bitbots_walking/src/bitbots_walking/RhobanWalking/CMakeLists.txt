set_source_files_properties(
    walking.pyx
    PROPERTIES CYTHON_IS_CXX TRUE
)

cython_add_module(walking walking.pyx walking.cpp CubicSpline.cpp HumanoidModel.cpp
        IKWalk.cpp LegIK.cpp Polynom.cpp Spline.cpp)

set_target_properties(walking PROPERTIES COMPILE_FLAGS -fPIC
                        LINK_FLAGS -fPIC)

set(INSTALL_TARGETS ${INSTALL_TARGETS} walking)

set_target_properties(walking
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/RhobanWalking/)


# Die kompilierten Module installieren
install(
    TARGETS ${INSTALL_TARGETS}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/bitbots_walking/RhobanWalking/
)

