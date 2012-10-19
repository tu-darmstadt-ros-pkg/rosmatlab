# - This module looks for Matlab and associated development libraries
# Defines:
#  MATLAB_INCLUDE_DIR:      include path for mex.h, engine.h
#  MATLAB_LIBRARIES:        required libraries: libmex, etc
#  MATLAB_MEX_LIBRARY:      path to libmex.lib
#  MATLAB_MX_LIBRARY:       path to libmx.lib
#  MATLAB_ENG_LIBRARY:      path to libeng.lib
#  MATLAB_MEX_VERSION_FILE: path to mexversion.rc or mexversion.c
#  MATLAB_MEX_SUFFIX:       filename suffix for mex-files (e.g. '.mexglx' or '.mexw64')
#  MATLAB_ROOT:             path to the Matlab root
#  MATLAB_SYS:              path to the Matlab system binary dir
#  MATLAB_BIN:              path to the Matlab binary dir

# This version modified by RW Penney, November 2008
# $Revision: 27 $, $Date: 2008-12-22 11:47:45 +0000 (Mon, 22 Dec 2008) $
# modified by Johannes Meyer, October 2012

if(MATLAB_FOUND)
  return()
endif(MATLAB_FOUND)

UNSET(MATLAB_FOUND CACHE)
UNSET(MATLAB_INCLUDE_DIR CACHE)
UNSET(MATLAB_MEX_LIBRARY CACHE)

IF(WIN32)

  FILE(GLOB _auto_matlab_prefixes "C:/Program*/MATLAB*/R20*")

  IF(CMAKE_SIZEOF_VOID_P EQUAL 4)
    # Regular x86
    SET(MATLAB_MEX_SUFFIX mexw32)
    SET(MATLAB_ARCH "win32")
  ELSE(CMAKE_SIZEOF_VOID_P EQUAL 4)
    SET(MATLAB_MEX_SUFFIX mexw64)
    SET(MATLAB_ARCH "win64")
  ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 4)

  # Search for available compilers:
  # (This would be neater using 'ELSEIF', but that isn't available until cmake-2.4.4)
  IF(${CMAKE_GENERATOR} MATCHES "Visual Studio 6")
    SET(_matlab_path_suffixes "extern/lib/${MATLAB_ARCH}/microsoft/msvc60"
        "extern/lib/${MATLAB_ARCH}/microsoft")
  ENDIF(${CMAKE_GENERATOR} MATCHES "Visual Studio 6")
  IF(${CMAKE_GENERATOR} MATCHES "Visual Studio 7")
    SET(_matlab_path_suffixes "extern/lib/${MATLAB_ARCH}/microsoft/msvc70"
        "extern/lib/${MATLAB_ARCH}/microsoft")
  ENDIF(${CMAKE_GENERATOR} MATCHES "Visual Studio 7")
  IF(${CMAKE_GENERATOR} MATCHES "Visual Studio [891]*")
    SET(_matlab_path_suffixes "extern/lib/${MATLAB_ARCH}/microsoft/msvc71"
        "extern/lib/${MATLAB_ARCH}/microsoft")
  ENDIF(${CMAKE_GENERATOR} MATCHES "Visual Studio [891]*")
  IF(${CMAKE_GENERATOR} MATCHES "Borland")
    SET(_matlab_path_suffixes "extern/lib/win32/microsoft/bcc54")
  ENDIF(${CMAKE_GENERATOR} MATCHES "Borland")
  IF(NOT _matlab_path_suffixes)
    MESSAGE(FATAL_ERROR "Generator not compatible: ${CMAKE_GENERATOR}")
  ENDIF(NOT _matlab_path_suffixes)

  SET(_libmex_name "libmex")
  SET(_libmx_name "libmx")
  SET(_libeng_name "libeng")

ELSE(WIN32)

  FILE(GLOB _auto_matlab_prefixes "/usr/local/matlab*/R20*" "/usr/local/MATLAB*/R20*" "/opt/matlab*/R20*" "/opt/MATLAB*/R20*")

  IF(CMAKE_SIZEOF_VOID_P EQUAL 4)
    # Regular x86
    SET(_matlab_path_suffixes "bin/glnx86")
    SET(MATLAB_ARCH glnx86)
    SET(MATLAB_MEX_SUFFIX mexglx)
  ELSE(CMAKE_SIZEOF_VOID_P EQUAL 4)
    SET(_matlab_path_suffixes "bin/glnxa64")
    SET(MATLAB_ARCH glnxa64)
    SET(MATLAB_MEX_SUFFIX mexa64)
  ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 4)

  SET(_libmex_name "mex")
  SET(_libmx_name "mx")
  SET(_libeng_name "eng")
ENDIF(WIN32)

SET(_matlab_path_prefixes
  ${MATLAB_PATH_PREFIXES}
  ${_auto_matlab_prefixes}
  ${MATLAB_ROOT}
)

# Search for include-files & libraries using architecture-dependent paths:
FOREACH(_matlab_prefix ${_matlab_path_prefixes})
  IF(NOT MATLAB_INCLUDE_DIR)
    #MESSAGE("Searching ${_matlab_prefix}")
    FIND_PATH(MATLAB_INCLUDE_DIR "mex.h"
      PATHS ${_matlab_prefix}/extern/include NO_DEFAULT_PATH)

    IF(MATLAB_INCLUDE_DIR)
      SET(MATLAB_ROOT ${_matlab_prefix}
        CACHE PATH "Matlab installation directory")
      SET(MATLAB_SYS "${_matlab_prefix}/sys/os/${MATLAB_ARCH}" CACHE PATH "Path to the Matlab system files")
      SET(MATLAB_BIN "${_matlab_prefix}/bin/${MATLAB_ARCH}" CACHE PATH "Path to the Matlab binaries")
      IF(WIN32)
        SET(MATLAB_MEX_VERSIONFILE "${_matlab_prefix}/extern/include/mexversion.rc")
      ELSE(WIN32)
        SET(MATLAB_MEX_VERSIONFILE "${_matlab_prefix}/extern/src/mexversion.c")
      ENDIF(WIN32)

      MESSAGE(STATUS "Found Matlab includes in ${MATLAB_INCLUDE_DIR}")

      FOREACH(_matlab_path_suffix ${_matlab_path_suffixes})
        SET(_matlab_libdir ${_matlab_prefix}/${_matlab_path_suffix})
        #MESSAGE("Searching ${_matlab_prefix} ... ${_matlab_libdir}")
        IF(NOT MATLAB_MEX_LIBRARY)
          FIND_LIBRARY(MATLAB_MEX_LIBRARY ${_libmex_name} PATHS ${_matlab_libdir} NO_DEFAULT_PATH)
          FIND_LIBRARY(MATLAB_MX_LIBRARY ${_libmx_name} PATHS ${_matlab_libdir} NO_DEFAULT_PATH)
          FIND_LIBRARY(MATLAB_ENG_LIBRARY ${_libeng_name} PATHS ${_matlab_libdir} NO_DEFAULT_PATH)

          IF(MATLAB_MEX_LIBRARY)
            MESSAGE(STATUS "Found Matlab libraries in ${_matlab_libdir}")
          ENDIF(MATLAB_MEX_LIBRARY)
        ENDIF(NOT MATLAB_MEX_LIBRARY)
      ENDFOREACH(_matlab_path_suffix)
    ENDIF(MATLAB_INCLUDE_DIR)
  ENDIF(NOT MATLAB_INCLUDE_DIR)
ENDFOREACH(_matlab_prefix)

SET(MATLAB_LIBRARIES
  ${MATLAB_MEX_LIBRARY}
  ${MATLAB_MX_LIBRARY}
  ${MATLAB_ENG_LIBRARY}
)

IF(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)
  SET(MATLAB_FOUND 1)
ENDIF(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)

MARK_AS_ADVANCED(
  MATLAB_LIBRARIES
  MATLAB_ARCH
  MATLAB_SYS
  MATLAB_MEX_LIBRARY
  MATLAB_MX_LIBRARY
  MATLAB_ENG_LIBRARY
  MATLAB_INCLUDE_DIR
  MATLAB_MEX_SUFFIX
  MATLAB_MEX_VERSIONFILE
  MATLAB_FOUND
)
