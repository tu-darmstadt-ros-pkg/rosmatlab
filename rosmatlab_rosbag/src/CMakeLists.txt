###########
## Build ##
###########

add_library(rosmatlab_rosbag SHARED bag.cpp view.cpp)
target_link_libraries(rosmatlab_rosbag ${catkin_LIBRARIES})

add_mex(rosmatlab_rosbag_mex mex.cpp OUTPUT_NAME internal DESTINATION +rosbag/@Bag/private)
target_link_libraries(rosmatlab_rosbag_mex rosmatlab_rosbag)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS rosmatlab_rosbag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
