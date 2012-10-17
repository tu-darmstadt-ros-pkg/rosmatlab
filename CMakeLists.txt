project(ros_matlab)
cmake_minimum_required(VERSION 2.8)
find_package(catkin REQUIRED)
catkin_stack()

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND DEFINED ENV{ROS_WORKSPACE})
  set(CMAKE_INSTALL_PREFIX "$ENV{ROS_WORKSPACE}/install" CACHE PATH "Install prefix" FORCE)
endif()

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
