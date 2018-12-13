find_path(Eigen_INCLUDE_DIR Eigen/Core
    PATHS /usr/include/eigen3 /usr/local/include/eigen3)

set(Eigen_INCLUDE_DIRS ${Eigen_INCLUDE_DIR} ${Eigen_INCLUDE_DIR}/Eigen)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen DEFAULT_MSG Eigen_INCLUDE_DIR)

mark_as_advanced(Eigen_INCLUDE_DIR)

