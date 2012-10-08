#include <mex.h>
#include <ros/ros.h>
#include "ros_init.h"

bool initialized = false;
ros::NodeHandle *node_handle = 0;

void ros_init()
{
  if (!initialized) {
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = 0;
      ros::init(argc, argv, "matlab", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
      mexPrintf("[rosmatlab] Initialized ROS environment, node name is %s", ros::this_node::getName().c_str());
    }

    if (!node_handle) {
      node_handle = new ros::NodeHandle();
      mexPrintf("[rosmatlab] Created new NodeHandle");
    }
  }
}

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  ros_init();
  if (nlhs > 0) {
    plhs[0] = mxCreateLogicalScalar(ros::isInitialized());
  }
}
