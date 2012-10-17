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

#include <rosmatlab/init.h>
#include <rosmatlab/exception.h>

namespace rosmatlab {

  ros::NodeHandle *node_handle_ = 0;
  ros::AsyncSpinner *spinner_ = 0;

  void init()
  {
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = 0;
      ros::init(argc, argv, "matlab", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    }

    if (!node_handle_) {
      node_handle_ = new ros::NodeHandle();
    }

    if (!spinner_) {
      spinner_ = new ros::AsyncSpinner(1);
      spinner_->start();
    }
  }

  void shutdown() {
    delete node_handle_;
    node_handle_ = 0;
    delete spinner_;
    spinner_ = 0;
    ros::shutdown();
  }

  ros::NodeHandle &nodeHandle() {
    if (!node_handle_) throw Exception("rosmatlab is not initalized");
    return *node_handle_;
  }

} // namespace rosmatlab

