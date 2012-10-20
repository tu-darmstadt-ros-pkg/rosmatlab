cmake_minimum_required(VERSION 2.8.3)
project(rosmatlab)

find_package(Matlab REQUIRED)
set(${PROJECT_NAME}_INCLUDE_DIRS ${MATLAB_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} include)
include_directories(${${PROJECT_NAME}_INCLUDE_DIRS})

find_package(catkin REQUIRED)
catkin_project(rosmatlab
  INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS}
  LIBRARIES rosmatlab
  DEPENDS cpp_introspection roscpp
  CFG_EXTRAS rosmatlab.cmake
)
include(${CMAKE_BINARY_DIR}/cmake/${PROJECT_NAME}/rosmatlab.cmake)

add_subdirectory(src)

configure_file(matlab.in matlab @ONLY)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/matlab DESTINATION bin)
configure_file(setup.m.in setup.m @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.m DESTINATION share/${PROJECT_NAME})
install(DIRECTORY matlab/ DESTINATION ${PROJECT_MATLAB_INSTALL_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

