
set_source_files_properties(
    controller.pyx register_tables.pyx converter.pyx
    PROPERTIES CYTHON_IS_CXX TRUE
)


# Modul für die LowLevel-Kommunikation mit dem cm730 und mx28
cython_add_module(controller controller.pyx)
cython_add_module(converter converter.pyx)
cython_add_module(register_tables  register_tables.pyx)


# Controller linken
target_link_libraries(controller)

set_target_properties(controller converter register_tables
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/lowlevel/controller)


install(
    TARGETS controller converter register_tables
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/bitbots_cm730/lowlevel/controller
)

