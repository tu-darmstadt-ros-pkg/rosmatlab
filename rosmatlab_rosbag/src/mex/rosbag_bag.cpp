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
#include <rosmatlab/rosbag/bag.h>

using namespace rosmatlab;
using namespace rosmatlab::rosbag;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  static MexMethodMap<Bag> methods;
  if (!methods.initialize()) {
    methods
    .add("open",              &Bag::open)
    .add("close",             &Bag::close)
    .add("write",             &Bag::write)
    .add("data",              &Bag::data)

    .add("getFileName",       &Bag::getFileName)
    .add("getMode",           &Bag::getMode)
    .add("getMajorVersion",   &Bag::getMajorVersion)
    .add("getMinorVersion",   &Bag::getMinorVersion)
    .add("getSize",           &Bag::getSize)
    .add("setCompression",    &Bag::setCompression)
    .add("getCompression",    &Bag::getCompression)
    .add("setChunkThreshold", &Bag::setChunkThreshold)
    .add("getChunkThreshold", &Bag::getChunkThreshold)
    .throwOnUnknown();
  }

  try {
    mexClassHelper<Bag>(nlhs, plhs, nrhs, prhs, methods);
  } catch (Exception& e) {
    mexErrMsgTxt(e.what());
  }
}
