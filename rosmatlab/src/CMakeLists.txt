add_library(rosmatlab SHARED init.cpp publisher.cpp subscriber.cpp param.cpp conversion.cpp options.cpp log.cpp exception.cpp connection_header.cpp)
target_link_libraries(rosmatlab ${catkin_LIBRARIES})
install(TARGETS rosmatlab DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_subdirectory(mex)
