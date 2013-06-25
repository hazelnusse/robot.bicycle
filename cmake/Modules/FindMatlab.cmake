
set(MATLAB_ROOT_BIN ${MATLAB_ROOT}/bin/glnxa64)

find_library(MATLAB_MEX_LIBRARY
             mex
             ${MATLAB_ROOT_BIN})

find_library(MATLAB_MX_LIBRARY
             mx
             ${MATLAB_ROOT_BIN})

find_library(MATLAB_ENG_LIBRARY
             eng
             ${MATLAB_ROOT_BIN})

find_library(MATLAB_UT_LIBRARY
             ut 
             ${MATLAB_ROOT_BIN})

find_library(MATLAB_MAT_LIBRARY
             mat
             ${MATLAB_ROOT_BIN})

find_path(MATLAB_INCLUDE_DIR 
          "mex.h"
          ${MATLAB_ROOT}/extern/include)

set(MATLAB_LIBRARIES
  ${MATLAB_MEX_LIBRARY}
  ${MATLAB_MX_LIBRARY}
  ${MATLAB_ENG_LIBRARY}
  ${MATLAB_UT_LIBRARY}
  ${MATLAB_MAT_LIBRARY}
)

if(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)
  set(MATLAB_FOUND 1)
endif()

mark_as_advanced(
  MATLAB_LIBRARIES
  MATLAB_MEX_LIBRARY
  MATLAB_MX_LIBRARY
  MATLAB_MAT_LIBRARY
  MATLAB_UT_LIBRARY
  MATLAB_ENG_LIBRARY
  MATLAB_INCLUDE_DIR
  MATLAB_FOUND
  MATLAB_ROOT)

