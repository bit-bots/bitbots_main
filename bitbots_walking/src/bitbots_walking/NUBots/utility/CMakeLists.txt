# This directory contains our message types and other shared types
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared/)
INCLUDE_DIRECTORIES(..)
# This gets all of our source files
FILE(GLOB_RECURSE src "*/**.cpp" "*/**.c" "*/**.h")

# Build a library from these files
ADD_LIBRARY(utility ${src})
set_target_properties(utility PROPERTIES COMPILE_FLAGS "-fPIC"
                        LINK_FLAGS -fPIC)

#target_link_libraries(utility ${BLAS_LIBRARIES})
target_link_libraries(utility ${CMAKE_INSTALL_PREFIX}/lib/libarmadillo.so ${ARMADILLO_LIBRARIES})
# Put it in an IDE group for shared
SET_PROPERTY(TARGET utility PROPERTY FOLDER "shared/")

install(TARGETS utility LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/bitbots_walking/)

add_subdirectory(motion)