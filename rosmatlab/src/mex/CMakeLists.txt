add_mex(ros_init ros_init.cpp OUTPUT_NAME init DESTINATION +ros)
target_link_libraries(ros_init rosmatlab)

add_mex(ros_shutdown ros_shutdown.cpp OUTPUT_NAME shutdown DESTINATION +ros)
target_link_libraries(ros_shutdown rosmatlab)

add_mex(ros_now ros_now.cpp OUTPUT_NAME now DESTINATION +ros)
target_link_libraries(ros_now rosmatlab)

add_mex(ros_log ros_log.cpp OUTPUT_NAME log DESTINATION +ros)
target_link_libraries(ros_log rosmatlab)

add_mex(ros_subscriber ros_subscriber.cpp OUTPUT_NAME internal DESTINATION +ros/@Subscriber/private)
target_link_libraries(ros_subscriber rosmatlab)

add_mex(ros_publisher ros_publisher.cpp OUTPUT_NAME internal DESTINATION +ros/@Publisher/private)
target_link_libraries(ros_publisher rosmatlab)

add_mex(ros_param ros_param.cpp OUTPUT_NAME param DESTINATION +ros)
target_link_libraries(ros_param rosmatlab)

add_mex(ros_param_get ros_param_get.cpp OUTPUT_NAME get DESTINATION +ros/+param)
target_link_libraries(ros_param_get rosmatlab)

add_mex(ros_param_set ros_param_set.cpp OUTPUT_NAME set DESTINATION +ros/+param)
target_link_libraries(ros_param_set rosmatlab)

add_mex(ros_param_delete ros_param_delete.cpp OUTPUT_NAME delete DESTINATION +ros/+param)
target_link_libraries(ros_param_delete rosmatlab)

add_mex(ros_param_has ros_param_has.cpp OUTPUT_NAME has DESTINATION +ros/+param)
target_link_libraries(ros_param_has rosmatlab)
