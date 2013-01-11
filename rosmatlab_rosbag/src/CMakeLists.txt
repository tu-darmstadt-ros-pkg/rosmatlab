###########
## Build ##
###########

add_library(rosmatlab_rosbag SHARED bag.cpp view.cpp)
target_link_libraries(rosmatlab_rosbag ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS rosmatlab_rosbag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(mex)
