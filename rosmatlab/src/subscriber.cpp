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
#include <rosmatlab/options.h>
#include <rosmatlab/conversion.h>
#include <rosmatlab/log.h>
#include <rosmatlab/connection_header.h>

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
  const std::type_info& getTypeInfo() { return subscriber_->introspection_->getTypeId(); }
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

mxArray *Subscriber::subscribe(int nrhs, const mxArray *prhs[]) {
  if (nrhs < 2) {
    throw ArgumentException("Subscriber.subscribe", 2);
  }

  options_ = ros::SubscribeOptions();
  for(int i = 0; i < nrhs; i++) {
    switch(i) {
      case 0:
        if (!Options::isString(prhs[i])) throw Exception("Subscriber.subscribe", "need a topic as 1st argument");
        options_.topic = Options::getString(prhs[i]);
        break;

      case 1:
        if (!Options::isString(prhs[i])) throw Exception("Subscriber.subscribe", "need a datatype as 2nd argument");
        options_.datatype = Options::getString(prhs[i]);
        break;

      case 2:
        if (!Options::isDoubleScalar(prhs[i])) throw Exception("Subscriber.subscribe", "need a queue size as 3rd argument");
        options_.queue_size = Options::getDoubleScalar(prhs[i]);
        break;

      default:
        throw ArgumentException("Subscriber.subscribe", "too many arguments");
    }
  }

  introspection_ = cpp_introspection::messageByDataType(options_.datatype);
  if (!introspection_) throw Exception("Subscriber.subscribe", "unknown datatype '" + options_.datatype + "'");
  options_.md5sum = introspection_->getMD5Sum();
  options_.helper.reset(new SubscriptionCallbackHelper(this));

  *this = node_handle_.subscribe(options_);
  return mxCreateLogicalScalar(*this);
}

mxArray *Subscriber::poll(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  ros::WallDuration timeout = timeout_;
  if (nrhs && mxIsDouble(*prhs) && mxGetPr(*prhs)) { timeout.fromSec(*mxGetPr(*prhs++)); nrhs--; }
  callback_queue_.callOne(timeout);

  last_event_.reset();
  if (!new_event_) {
    plhs[0] = mxCreateStructMatrix(0,0,0,0);
    return plhs[0];
  }

  plhs[0] = Conversion(introspect(new_event_->getConstMessage())).toMatlab();
  last_event_.swap(new_event_);

  if (nlhs > 1) plhs[1] = getConnectionHeader();
  if (nlhs > 2) plhs[2] = getReceiptTime();
  return plhs[0];
}

mxArray *Subscriber::getConnectionHeader() const
{
  if (!last_event_) return mxCreateStructMatrix(0, 0, 0, 0);
  return ConnectionHeader(last_event_->getConnectionHeaderPtr()).toMatlab();
}

mxArray *Subscriber::getReceiptTime() const
{
  mxArray *receiptTime = mxCreateDoubleScalar(0);
  if (!last_event_) return receiptTime;
  *mxGetPr(receiptTime) = last_event_->getReceiptTime().toSec();
  return receiptTime;
}

mxArray *Subscriber::getTopic() const
{
  return mxCreateString(ros::Subscriber::getTopic().c_str());
}

mxArray *Subscriber::getDataType() const
{
  if (!introspection_) return mxCreateString(0);
  return mxCreateString(introspection_->getDataType());
}

mxArray *Subscriber::getMD5Sum() const
{
  if (!introspection_) return mxCreateString(0);
  return mxCreateString(introspection_->getMD5Sum());
}

mxArray *Subscriber::getNumPublishers() const
{
  return mxCreateDoubleScalar(ros::Subscriber::getNumPublishers());
}

MessagePtr Subscriber::introspect(const VoidConstPtr& msg) {
  if (!introspection_ || !msg) return MessagePtr();
  return introspection_->introspect(msg.get());
}

void Subscriber::callback(const ros::MessageEvent<void>& event)
{
  if (new_event_) {
    ROSMATLAB_WARN("missed a %s message on topic %s, polling is too slow...", options_.datatype.c_str(), options_.topic.c_str());
    return;
  }
  new_event_.reset(new MessageEvent(event));
}

VoidConstPtr SubscriptionCallbackHelper::deserialize(const ros::SubscriptionCallbackHelperDeserializeParams& params)
{
  ros::serialization::IStream stream(params.buffer, params.length);
  VoidPtr msg = subscriber_->introspection_->deserialize(stream);
  if (!msg) ROSMATLAB_WARN("deserialization of a message of type %s failed", subscriber_->options_.datatype.c_str());

  return VoidConstPtr(msg);
}

void SubscriptionCallbackHelper::call(ros::SubscriptionCallbackHelperCallParams& params)
{
  ros::MessageEvent<void> event(params.event, boost::bind(&cpp_introspection::Message::createInstance, subscriber_->introspection_));
  subscriber_->callback(event);
}

} // namespace rosmatlab
