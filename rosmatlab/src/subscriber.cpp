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

#include <rosmatlab/subscriber.h>
#include <rosmatlab/exception.h>
#include <rosmatlab/string.h>
#include <rosmatlab/conversion.h>

#include <introspection/message.h>

namespace rosmatlab {

template <> const char *Object<Subscriber>::class_name_ = "ros.Subscriber";
static const ros::WallDuration DEFAULT_TIMEOUT(1e-3);

class SubscriptionCallbackHelper : public ros::SubscriptionCallbackHelper
{
public:
  SubscriptionCallbackHelper(Subscriber *subscriber)
    : subscriber_(subscriber) {}
  virtual ~SubscriptionCallbackHelper() {}

  VoidConstPtr deserialize(const ros::SubscriptionCallbackHelperDeserializeParams&);
  void call(ros::SubscriptionCallbackHelperCallParams& params);
  const std::type_info& getTypeInfo() { return subscriber_->message_->getTypeId(); }
  bool isConst() { return false; }

private:
  Subscriber *subscriber_;
};

Subscriber::Subscriber()
  : Object<Subscriber>(this)
{
  timeout_ = DEFAULT_TIMEOUT;
  node_handle_.setCallbackQueue(&callback_queue_);
}

Subscriber::Subscriber(int nrhs, const mxArray *prhs[])
  : Object<Subscriber>(this)
{
  timeout_ = DEFAULT_TIMEOUT;
  node_handle_.setCallbackQueue(&callback_queue_);

  if (nrhs > 0) subscribe(nrhs, prhs);
}

Subscriber::~Subscriber() {
  shutdown();
}

bool Subscriber::subscribe(int nrhs, const mxArray *prhs[]) {
  if (nrhs < 2) {
    throw Exception("Subscriber::subscribe needs at least two arguments");
  }

  options_.topic = getString(prhs[0]);
  options_.datatype = getString(prhs[1]);
  if (nrhs >= 3 && mxIsDouble(prhs[2])) options_.queue_size = *mxGetPr(prhs[2]);

  message_ = cpp_introspection::messageByDataType(options_.datatype);
  if (!message_) throw Exception("ros.Subscriber: unknown datatype '" + options_.datatype + "'");
  options_.md5sum = message_->getMD5Sum();
  options_.helper.reset(new SubscriptionCallbackHelper(this));

  *this = node_handle_.subscribe(options_);
  return *this;
}

VoidConstPtr Subscriber::poll(int nrhs, const mxArray *prhs[])
{
  ros::WallDuration timeout = timeout_;
  if (nrhs && mxIsDouble(*prhs) && mxGetPr(*prhs)) { timeout.fromSec(*mxGetPr(*prhs++)); nrhs--; }
  callback_queue_.callOne(timeout);
  VoidConstPtr copy = msg_;
  msg_.reset();
  return copy;
}

mxArray *Subscriber::poll(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  VoidConstPtr msg = poll(nrhs, prhs);
  mxArray *result = 0;

  if (msg)
    result = Conversion(introspect(msg)).toMatlab();
  else
    result = mxCreateStructMatrix(0,0,0,0);

  if (nlhs > 0) plhs[0] = result;
  return result;
}

MessagePtr Subscriber::introspect(const VoidConstPtr& msg) {
  if (!message_ || !msg) return message_;
  return message_->introspect(msg.get());
}

void Subscriber::callback(const ros::MessageEvent<void>& event)
{
  if (msg_) {
    ROS_WARN_NAMED("rosmatlab", "missed a %s message on topic %s, polling is too slow...", options_.datatype.c_str(), options_.topic.c_str());
    return;
  }
  msg_ = event.getMessage();
}

ros::CallbackQueueInterface* Subscriber::getCallbackQueue() {
  return &callback_queue_;
}

VoidConstPtr SubscriptionCallbackHelper::deserialize(const ros::SubscriptionCallbackHelperDeserializeParams& params)
{
  ros::serialization::IStream stream(params.buffer, params.length);
  VoidPtr msg = subscriber_->message_->deserialize(stream);
  if (!msg) ROS_WARN_NAMED("rosmatlab", "deserialization of a message of type %s failed", subscriber_->options_.datatype.c_str());

  // TODO: setConnectionHeader
  return VoidConstPtr(msg);
}

void SubscriptionCallbackHelper::call(ros::SubscriptionCallbackHelperCallParams& params)
{
  ros::MessageEvent<void> event(params.event, boost::bind(&cpp_introspection::Message::createInstance, subscriber_->message_));
  subscriber_->callback(event);
}

} // namespace rosmatlab
