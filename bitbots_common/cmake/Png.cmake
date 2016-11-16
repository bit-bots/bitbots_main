# This module defines
#  PNG_INCLUDE_DIR, where to find png.h, etc.
#  PNG_FOUND, If false, do not try to use PNG.
# also defined, but not for general use are
#  PNG_LIBRARY, where to find the PNG library.

FIND_PATH(PNG_INCLUDE_DIR png.h PATHS ${PNG_ROOT} PATH_SUFFIXES
    include include/libpng NO_DEFAULT_PATH
    )

FIND_PATH(PNG_INCLUDE_DIR png.h PATHS ${PNG_ROOT} PATH_SUFFIXES
    include include/libpng
    )

SET(PNG_NAMES ${PNG_NAMES} png libpng png12 libpng12)

FIND_LIBRARY(PNG_LIBRARY NAMES ${PNG_NAMES} PATHS ${PNG_ROOT}
  PATH_SUFFIXES  /lib NO_DEFAULT_PATHS
  )

FIND_LIBRARY(PNG_LIBRARY NAMES ${PNG_NAMES} PATHS ${PNG_ROOT}
  PATH_SUFFIXES /lib
  )

# handle the QUIETLY and REQUIRED arguments and set PNG_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PNG DEFAULT_MSG PNG_LIBRARY PNG_INCLUDE_DIR)
