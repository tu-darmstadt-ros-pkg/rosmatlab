add_mex(rosmatlab_mex_rosbag_bag rosbag_bag.cpp OUTPUT_NAME internal DESTINATION +rosbag/@Bag/private)
target_link_libraries(rosmatlab_mex_rosbag_bag rosmatlab_rosbag)

add_mex(rosmatlab_mex_rosbag_view rosbag_view.cpp OUTPUT_NAME internal DESTINATION +rosbag/@View/private)
target_link_libraries(rosmatlab_mex_rosbag_view rosmatlab_rosbag)
