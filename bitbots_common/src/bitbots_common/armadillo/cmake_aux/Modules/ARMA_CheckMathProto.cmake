# - Check if the prototype for a single argument math function exists.
# ARMA_CHECK_MATH_PROTO (FUNCTION NAMESPACE HEADER VARIABLE)
#
#  FUNCTION  - the name of the single argument math function you are looking for
#  NAMESPACE - the name of the namespace
#  HEADER    - the header(s) where the prototype should be declared
#  VARIABLE  - variable to store the result
#
# The following variables may be set before calling this macro to
# modify the way the check is run:
#
#  CMAKE_REQUIRED_FLAGS = string of compile command line flags
#  CMAKE_REQUIRED_DEFINITIONS = list of macros to define (-DFOO=bar)
#  CMAKE_REQUIRED_INCLUDES = list of include directories

# adapted from "CheckPrototypeExists.cmake"
# ( http://websvn.kde.org/trunk/KDE/kdelibs/cmake/modules/CheckPrototypeExists.cmake )
# on 2009-06-19 by Conrad Sanderson (conradsand at ieee dot org)

# original copyright for "CheckPrototypeExists.cmake":
#
# Copyright (c) 2006, Alexander Neundorf, <neundorf@kde.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.


INCLUDE(CheckCXXSourceCompiles)

MACRO (ARMA_CHECK_MATH_PROTO _SYMBOL _NAMESPACE _HEADER _RESULT)

  SET(_INCLUDE_FILES)

  FOREACH (it ${_HEADER})
    SET(_INCLUDE_FILES "${_INCLUDE_FILES}#include <${it}>\n")
  ENDFOREACH (it)
   
  SET(_TMP_SOURCE_CODE "
${_INCLUDE_FILES}
int main()
  {
  #if !defined(${_SYMBOL})
    int i = (${_NAMESPACE}::${_SYMBOL})(1.0);
  #endif
  return 0;
  }
")

  CHECK_CXX_SOURCE_COMPILES("${_TMP_SOURCE_CODE}" ${_RESULT})

ENDMACRO (ARMA_CHECK_MATH_PROTO _SYMBOL _NAMESPACE _HEADER _RESULT)
