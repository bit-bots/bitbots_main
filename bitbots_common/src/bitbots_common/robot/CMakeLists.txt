set_source_files_properties(
    pypose.pyx kinematics.pyx comcalculator.pyx
    PROPERTIES CYTHON_IS_CXX TRUE
)

include_directories(../utilCython)
include_directories(../pose)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(.)

cython_add_module(kinematics kinematics.pyx )

set(INSTALL_TARGETS kinematics)

if(BUILD_COM_CALCULATOR)
    cython_add_module(comcalculator comcalculator.pyx com_calculator.cpp
                        kinematic_robot.cpp)
    target_link_libraries(comcalculator debug-common)

    set(INSTALL_TARGETS ${INSTALL_TARGETS} comcalculator)
endif(BUILD_COM_CALCULATOR)

#Set some options for creating the library
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic -DNPRINT_DEBUG -Wno-enum-compare")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNPRINT_DEBUG")

# Remove the -ffast-math from the c++ flags for the kinematic module, because it causes NaN occurrences and invalid behaviour
STRING(REPLACE "-ffast-math" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
STRING(REPLACE "-ffast-math" "" CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})
#STRING(REPLACE "-ffast-math" "" CMAKE_CXX_FLAGS_RELEASEWITHDEBINF ${CMAKE_CXX_FLAGS_RELEASEWITHDEBINF})


if(${YAMLPATH} MATCHES YAMLPATH-NOTFOUND)
    set(YAML_INCLUDE "")
else()
    set(YAML_INCLUDE load_from_config.cpp)
endif()

#create library, so the tests can link without recompiling the whole module
add_library(robot STATIC kinematic_robot.cpp  kinematic_task.cpp
                    robot.pxi ${YAML_INCLUDE})

set_target_properties(robot PROPERTIES COMPILE_FLAGS "-fPIC"
                        LINK_FLAGS -fPIC)

#if(NOT ${YAMLPATH} MATCHES YAMLPATH-NOTFOUND)
#    target_link_libraries(robot yaml-cpp)
#endif()
target_link_libraries(kinematics robot)

set_target_properties(${INSTALL_TARGETS}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/robot)


install(
    TARGETS ${INSTALL_TARGETS}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/bitbots_common/robot
)

if(ENABLE_GTESTS)
    add_subdirectory(test)
endif(ENABLE_GTESTS)
