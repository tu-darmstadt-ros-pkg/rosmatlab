cmake_minimum_required(VERSION 2.8.3)
project(rosmatlab_common_msgs)

find_package(catkin REQUIRED rosmatlab)
catkin_package(
  CATKIN_DEPENDS rosmatlab
)

include_directories(${catkin_INCLUDE_DIRS})

add_mex_messages(std_msgs)
add_mex_messages(rosgraph_msgs)

add_mex_messages(actionlib_msgs)
add_mex_messages(diagnostic_msgs)
add_mex_messages(geometry_msgs)
add_mex_messages(nav_msgs)
add_mex_messages(sensor_msgs)
add_mex_messages(shape_msgs)
add_mex_messages(stereo_msgs)
add_mex_messages(trajectory_msgs)
add_mex_messages(visualization_msgs)
