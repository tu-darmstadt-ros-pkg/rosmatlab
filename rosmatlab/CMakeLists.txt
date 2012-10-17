cmake_minimum_required(VERSION 2.8)
project(rosmatlab)

include(cmake/FindMatlab.cmake)
find_package(Matlab REQUIRED)
find_package(Boost REQUIRED)

set(${PROJECT_NAME}_INCLUDE_DIRS ${MATLAB_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} include)
include_directories(${${PROJECT_NAME}_INCLUDE_DIRS})

#configure_file(cmake/rosmatlab.cmake.in ${PROJECT_SOURCE_DIR}/cmake/rosmatlab.cmake @ONLY)

find_package(catkin REQUIRED)
catkin_project(rosmatlab
  INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS}
  LIBRARIES rosmatlab
  DEPENDS cpp_introspection roscpp
  CFG_EXTRAS rosmatlab.cmake
)

#set the default path for built executables to the "bin" directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_BINARY_DIR}/cmake/${PROJECT_NAME})
add_subdirectory(src)
