set_source_files_properties(
    InverseKinematicsPy.pyx
    PROPERTIES CYTHON_IS_CXX TRUE
)

include_directories(.)
include_directories(~/catkin_ws/src/bitbots_misc/bitbots_common/armadillo)

cython_add_module(InverseKinematicsPy InverseKinematicsPy.pyx)

set_target_properties(InverseKinematicsPy PROPERTIES COMPILE_FLAGS -fPIC
                        LINK_FLAGS -fPIC)

set(INSTALL_TARGETS InverseKinematicsPy)


target_link_libraries(InverseKinematicsPy utility)

install(
    TARGETS ${INSTALL_TARGETS}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/bitbots_walking/
)