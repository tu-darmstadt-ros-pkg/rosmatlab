#include <mex.h>
#include <ros/ros.h>
#include "ros_init.h"

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  ros_init();

  if (nlhs < 1) {
    mexErrMsgTxt("At least one output required!");
  }

  plhs[0] = mxCreateDoubleScalar(ros::Time::now().toSec());
}
