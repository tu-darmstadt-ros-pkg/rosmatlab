add_mex(ros_init ros_init.cpp OUTPUT_NAME init PACKAGE +ros)
target_link_libraries(ros_init rosmatlab)

add_mex(ros_shutdown ros_shutdown.cpp OUTPUT_NAME shutdown PACKAGE +ros)
target_link_libraries(ros_shutdown rosmatlab)

add_mex(ros_now ros_now.cpp OUTPUT_NAME now PACKAGE +ros)
target_link_libraries(ros_now rosmatlab)

add_mex(ros_subscriber ros_subscriber.cpp OUTPUT_NAME mex PACKAGE +ros/@Subscriber/private)
target_link_libraries(ros_subscriber rosmatlab)

add_mex(ros_publisher ros_publisher.cpp OUTPUT_NAME mex PACKAGE +ros/@Publisher/private)
target_link_libraries(ros_publisher rosmatlab)

add_mex(ros_param ros_param.cpp OUTPUT_NAME param PACKAGE +ros)
target_link_libraries(ros_param rosmatlab)
