add_mex(ros_init ros_init.cpp)
target_link_libraries(ros_init rosmatlab)

add_mex(ros_shutdown ros_shutdown.cpp)
target_link_libraries(ros_shutdown rosmatlab)

add_mex(ros_now ros_now.cpp)
target_link_libraries(ros_now rosmatlab)

add_mex(ros_subscriber ros_subscriber.cpp)
target_link_libraries(ros_subscriber rosmatlab)

add_mex(ros_publisher ros_publisher.cpp)
target_link_libraries(ros_publisher rosmatlab)

add_mex(ros_param ros_param.cpp)
target_link_libraries(ros_param rosmatlab)


install(TARGETS ros_init ros_shutdown ros_now ros_subscriber ros_publisher ros_param DESTINATION mex)
