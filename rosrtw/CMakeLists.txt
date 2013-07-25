cmake_minimum_required(VERSION 2.8.3)
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

find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS ${rosrtw_INCLUDE_DIRS}
  CATKIN_DEPENDS rosmatlab
  CFG_EXTRAS rosrtw.cmake
)
