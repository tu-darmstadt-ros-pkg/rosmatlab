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

#ifndef ROSMATLAB_SUBSCRIBER_H
#define ROSMATLAB_SUBSCRIBER_H

#include <rosmatlab/object.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <introspection/forwards.h>

namespace rosmatlab {

using cpp_introspection::VoidPtr;
using cpp_introspection::VoidConstPtr;
using cpp_introspection::MessagePtr;

class Subscriber : public ros::Subscriber, public Object<Subscriber>
{
public:
  Subscriber();
  Subscriber(int nrhs, const mxArray *prhs[]);
  ~Subscriber();

  using ros::Subscriber::operator=;
  mxArray *subscribe(int nrhs, const mxArray *prhs[]);
  mxArray *poll(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

  mxArray *getTopic() const;
  mxArray *getDataType() const;
  mxArray *getMD5Sum() const;

  mxArray *getConnectionHeader() const;
  mxArray *getReceiptTime() const;

  mxArray *getNumPublishers() const;

private:
  friend class SubscriptionCallbackHelper;
  typedef ros::MessageEvent<void> MessageEvent;
  void callback(const MessageEvent& event);
  MessagePtr introspect(const VoidConstPtr& msg);

private:
  ros::NodeHandle node_handle_;
  ros::SubscribeOptions options_;
  ros::CallbackQueue callback_queue_;
  ros::WallDuration timeout_;

  cpp_introspection::MessagePtr introspection_;
  boost::shared_ptr<MessageEvent> new_event_;
  boost::shared_ptr<MessageEvent> last_event_;
};

} // namespace rosmatlab

#endif // ROSMATLAB_SUBSCRIBER_H
