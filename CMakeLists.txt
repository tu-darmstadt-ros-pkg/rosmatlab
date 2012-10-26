project(rosmatlab)
cmake_minimum_required(VERSION 2.8.3)
find_package(catkin REQUIRED)
catkin_stack()

# Find Boost version in the Matlab environment and add catkin env hook
unset(Boost_INCLUDE_DIR CACHE)
unset(Boost_LIBRARY_DIR CACHE)
unset(Boost_LIBRARY_DIRS CACHE)
set(Boost_NO_SYSTEM_PATHS TRUE)
set(BOOST_LIBRARYDIR ${MATLAB_BIN})
find_package(Boost REQUIRED)
catkin_add_env_hooks(cmake/boost SHELLS all)

foreach(subdir
    rosmatlab
    rosmatlab_common_msgs
    rosrtw
    )
  add_subdirectory(${subdir})
endforeach()

# install all manifests
install_matching_to_share(manifest.xml)

install(FILES stack.xml 
        DESTINATION share/${PROJECT_NAME})
