//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
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

#include <rosmatlab/mex.h>
#include <rosmatlab/rosbag/view.h>

using namespace rosmatlab;
using namespace rosmatlab::rosbag;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  static MexMethodMap<View> methods;
  if (!methods.initialize()) {
    methods
    .add("addQuery",             &View::addQuery)

    .add("reset",                &View::reset)
    .add("start",                &View::start)
    .add("valid",                &View::valid)
    .add("eof",                  &View::eof)
    .add("increment",            &View::increment)

    .add("get",                  &View::get)
    .add("data",                 &View::data)
    .add("next",                 &View::next)

    .add("getSize",              &View::getSize)
    .add("getTime",              &View::getTime)
    .add("getTopic",             &View::getTopic)
    .add("getDataType",          &View::getDataType)
    .add("getMD5Sum",            &View::getMD5Sum)
    .add("getMessageDefinition", &View::getMessageDefinition)
    .add("getConnectionHeader",  &View::getConnectionHeader)
    .add("getCallerId",          &View::getCallerId)
    .add("isLatching",           &View::isLatching)

    .add("getQueries",           &View::getQueries)
    .add("getConnections",       &View::getConnections)
    .add("getBeginTime",         &View::getBeginTime)
    .add("getEndTime",           &View::getEndTime)
    .throwOnUnknown();
  }

  try {
    mexClassHelper<View>(nlhs, plhs, nrhs, prhs, methods);
  } catch (Exception& e) {
    mexErrMsgTxt(e.what());
  }
}
