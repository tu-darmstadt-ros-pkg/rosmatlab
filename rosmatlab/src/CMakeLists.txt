add_mex(ros_init ros_init.cpp)
target_link_libraries(ros_init ${roscpp_LIBRARIES})

add_mex(ros_time ros_time.cpp)
target_link_libraries(ros_time ${roscpp_LIBRARIES} ros_init)


