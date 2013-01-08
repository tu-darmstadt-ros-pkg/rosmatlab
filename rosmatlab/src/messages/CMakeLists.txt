if(NOT DEFINED package)
  message(SEND_ERROR "You have to set the package variable before adding this subdirectory to your CMake project.")
  return()
endif()

# Return if package target already exists
if(TARGET mex_${package})
  return()
endif()

# Find all dependent message packages


# List all messages in package ${package}
execute_process(
  COMMAND rosmsg package ${package}
  OUTPUT_VARIABLE ${package}_MSGS
  RESULT_VARIABLE ${package}_MSGS_RESULT
)
if(NOT ${${package}_MSGS_RESULT} EQUAL 0)
  message(SEND_ERROR "Could not list all messages from package ${package}.")
  return()
endif()
string(REPLACE "\n" ";" ${package}_MSGS "${${package}_MSGS}")
string(REPLACE "${package}/" "" ${package}_MSGS "${${package}_MSGS}")
list(REMOVE_ITEM ${package}_MSGS "")

# Ignore packages with no messages
if(NOT ${package}_MSGS)
  return()
endif()

# Find (and build) introspection library
find_library(introspection_${package} PATH_SUFFIXES introspection)
if(NOT introspection_${package})
  introspection_add(${package})
endif()

# Set install RPATH
list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}" "${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/introspection")

# Iterate over all messages and generate mex
unset(_msgs_LIBRARIES)
foreach(msg ${${package}_MSGS})
  message(STATUS "Generating MEX for message ${package}/${msg}...")

  configure_file(mex_message.cpp.in mex_${msg}.cpp @ONLY)
  add_mex(mex_${package}_${msg} mex_${msg}.cpp PACKAGE +${package} OUTPUT_NAME ${msg})
  target_link_libraries(mex_${package}_${msg} ${rosmatlab_LIBRARIES} introspection_${package})
  list(APPEND _msgs_LIBRARIES mex_${package}_${msg})
endforeach()

configure_file(mex_package.cpp.in mex_${package}.cpp @ONLY)
add_mex(mex_${package} mex_${package}.cpp)
target_link_libraries(mex_${package} ${_msgs_LIBRARIES} introspection_${package})
set_target_properties(mex_${package} PROPERTIES OUTPUT_NAME ${package})

