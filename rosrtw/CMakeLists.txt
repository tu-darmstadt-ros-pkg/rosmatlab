cmake_minimum_required(VERSION 2.8)
project(rosrtw)

find_package(rosmatlab REQUIRED)
if(NOT MATLAB_FOUND)
  message(FATAL_ERROR "You cannot build this package without an installation of Matlab on your computer.")
endif()

set(rosrtw_INCLUDE_DIRS
  include
  ${MATLAB_ROOT}/extern/include
  ${MATLAB_ROOT}/simulink/include
  ${MATLAB_ROOT}/rtw/c/src
  ${MATLAB_ROOT}/rtw/c/src/ext_mode/common
)

configure_file(cmake/rosrtw.cmake.in ${PROJECT_SOURCE_DIR}/cmake/rosrtw.cmake @ONLY)

find_package(catkin REQUIRED)
catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS ${rosrtw_INCLUDE_DIRS}
  DEPENDS rosmatlab
  CFG_EXTRAS rosrtw.cmake
)
