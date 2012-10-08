cmake_minimum_required(VERSION 2.8)
project(rosmatlab)

include(cmake/FindMatlab.cmake)
find_package(Matlab REQUIRED)
if(NOT MATLAB_FOUND)
  message(FATAL_ERROR "You cannot build this package without an installation of Matlab on your computer.")
endif()

configure_file(cmake/rosmatlab.cmake.in ${PROJECT_SOURCE_DIR}/cmake/rosmatlab.cmake @ONLY)

find_package(catkin REQUIRED)
catkin_project(rosmatlab
  INCLUDE_DIRS ${MATLAB_INCLUDE_DIR}
  CFG_EXTRAS rosmatlab.cmake
)

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

include(cmake/rosmatlab.cmake)
add_subdirectory(src)

