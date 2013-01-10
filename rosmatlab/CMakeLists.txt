cmake_minimum_required(VERSION 2.8.3)
project(rosmatlab)

# Find an installation of Matlab
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
find_package(Matlab REQUIRED)

# Find Boost
find_package(Boost REQUIRED)

# Set exported include directories
set(${PROJECT_NAME}_INCLUDE_DIRS include ${MATLAB_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
include_directories(${${PROJECT_NAME}_INCLUDE_DIRS})

find_package(catkin REQUIRED cpp_introspection roscpp)
catkin_package(
  INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS}
  LIBRARIES rosmatlab
  DEPENDS cpp_introspection roscpp
  CFG_EXTRAS rosmatlab.cmake
)
include(${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/rosmatlab.cmake)
include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src)

configure_file(matlab.in matlab @ONLY)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/matlab DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
configure_file(setup.m.in setup.m @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.m DESTINATION .)

install(DIRECTORY matlab/ DESTINATION ${GLOBAL_MATLAB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY src/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src)

