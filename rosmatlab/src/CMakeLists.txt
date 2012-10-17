find_package(ROS REQUIRED COMPONENTS roscpp cpp_introspection)
include_directories(${ROS_INCLUDE_DIRS})

add_library(rosmatlab SHARED init.cpp publisher.cpp subscriber.cpp param.cpp conversion.cpp)
target_link_libraries(rosmatlab ${ROS_LIBRARIES})
install(TARGETS rosmatlab DESTINATION lib)

find_package(rosmatlab REQUIRED)
add_subdirectory(mex)
