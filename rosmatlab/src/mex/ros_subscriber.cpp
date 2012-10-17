//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <mex.h>
#include <rosmatlab/ros.h>
#include <rosmatlab/string.h>

using namespace rosmatlab;

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  if (nrhs < 1) {
    mexErrMsgTxt("ros_subscriber: At least 1 input argument is required");
  }

  try {
    init();
    Subscriber *subscriber = getObject<Subscriber>(*prhs++); nrhs--;
    std::string method;
    if (nrhs) { method = getString(*prhs++); nrhs--; }

    // construction
    if (method == "create") {
      delete subscriber;
      mexPrintf("ros_subscriber: Creating new subscriber object\n");
      subscriber = new Subscriber(nrhs, prhs);
      plhs[0] = subscriber->handle();
      return;
    }

    // destruction
    if (method == "delete") {
      mexPrintf("ros_subscriber: Deleting subscriber object\n");
      delete subscriber;
      return;
    }

    if (!subscriber) {
      throw Exception("ros_subscriber: subscriber instance not found");
    }

    // subscribe()
    if (method == "subscribe") {
      plhs[0] = mxCreateLogicalScalar(subscriber->subscribe(nrhs, prhs));
      return;
    }

    // poll()
    if (method == "poll") {
      plhs[0] = subscriber->poll(nlhs, plhs, nrhs, prhs);
      return;
    }

    // getTopic()
    if (method == "topic") {
      plhs[0] = mxCreateString(subscriber->getTopic().c_str());
      return;
    }

    // getNumPublishers()
    if (method == "numpublishers") {
      plhs[0] = mxCreateDoubleScalar(subscriber->getNumPublishers());
      return;
    }

    // unknown method exception
    throw Exception("ros_subscriber: unknown method '" + method + "'");

  } catch(Exception &e) {
    mexErrMsgTxt(e.what());
  }
}
