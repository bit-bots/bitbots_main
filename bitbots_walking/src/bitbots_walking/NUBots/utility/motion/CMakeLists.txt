set_source_files_properties(
    InverseKinematicsPy.pyx
    PROPERTIES CYTHON_IS_CXX TRUE
)

include_directories(.)

cython_add_module(InverseKinematicsPy InverseKinematicsPy.pyx)

set_target_properties(InverseKinematicsPy PROPERTIES COMPILE_FLAGS -fPIC
                        LINK_FLAGS -fPIC)

set(INSTALL_TARGETS ${INSTALL_TARGETS} InverseKinematicsPy)


target_link_libraries(InverseKinematicsPy debug-common utility)

install(
    TARGETS ${INSTALL_TARGETS}
    LIBRARY DESTINATION ${PYLIB_INSTALL_PATH}/NUBots/utility/motion
)