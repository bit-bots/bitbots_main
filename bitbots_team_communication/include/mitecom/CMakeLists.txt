set_source_files_properties(
    mitecom.pyx
    PROPERTIES CYTHON_IS_CXX TRUE
)

include_directories(.)

cython_add_module(mitecom mitecom.pyx mitecom.cpp mitecom-network.cpp
    mitecom-handler.cpp)

install(
    TARGETS mitecom
    LIBRARY DESTINATION ${PYLIB_INSTALL_PATH}/mitecom/
)
