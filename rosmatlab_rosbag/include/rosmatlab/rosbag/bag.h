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

#ifndef ROSMATLAB_ROSBAG_BAG_H
#define ROSMATLAB_ROSBAG_BAG_H

#include <rosbag/bag.h>
#include <rosmatlab/object.h>

namespace rosmatlab {
namespace rosbag {

class Bag : public ::rosbag::Bag, public Object<Bag> {
public:
  Bag();
  Bag(int nrhs, const mxArray *prhs[]);
  virtual ~Bag();

  void open(int nrhs, const mxArray *prhs[]);
  void close();

  void get(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

  mxArray *getFileName()     const;                      //!< Get the filename of the bag
  mxArray *getMode()         const;                      //!< Get the mode the bag is in
  mxArray *getMajorVersion() const;                      //!< Get the major-version of the open bag file
  mxArray *getMinorVersion() const;                      //!< Get the minor-version of the open bag file
  mxArray *getSize()         const;                      //!< Get the current size of the bag file (a lower bound)

  void     setCompression(int nrhs, const mxArray *prhs[]);     //!< Set the compression method to use for writing chunks
  mxArray *getCompression() const;                              //!< Get the compression method to use for writing chunks
  void     setChunkThreshold(int nrhs, const mxArray *prhs[]);  //!< Set the threshold for creating new chunks
  mxArray *getChunkThreshold() const;                           //!< Get the threshold for creating new chunks

  void write(int nrhs, const mxArray *prhs[]);
};

} // namespace rosbag
} // namespace rosmatlab

#endif // ROSMATLAB_ROSBAG_BAG_H
