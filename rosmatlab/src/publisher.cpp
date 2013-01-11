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

#include <rosmatlab/publisher.h>
#include <rosmatlab/exception.h>
#include <rosmatlab/options.h>
#include <rosmatlab/conversion.h>

#include <introspection/message.h>

#include <ros/topic_manager.h>

namespace rosmatlab {

template <> const char *Object<Publisher>::class_name_ = "ros.Publisher";

Publisher::Publisher()
  : Object<Publisher>(this)
{
}

Publisher::Publisher(int nrhs, const mxArray *prhs[])
  : Object<Publisher>(this)
{
  if (nrhs > 0) advertise(nrhs, prhs);
}

Publisher::~Publisher() {
  shutdown();
}

mxArray *Publisher::advertise(int nrhs, const mxArray *prhs[])
{
  if (nrhs < 2) {
    throw ArgumentException("Publisher.advertise", 2);
  }

  options_ = ros::AdvertiseOptions();
  for(int i = 0; i < nrhs; i++) {
    switch(i) {
      case 0:
        if (!Options::isString(prhs[i])) throw Exception("Publisher.advertise", "need a topic as 1st argument");
        options_.topic = Options::getString(prhs[i]);
        break;

      case 1:
        if (!Options::isString(prhs[i])) throw Exception("Publisher.advertise", "need a datatype as 2nd argument");
        options_.datatype = Options::getString(prhs[i]);
        break;

      case 2:
        if (!Options::isDoubleScalar(prhs[i])) throw Exception("Publisher.advertise", "need a queue size as 3rd argument (optional)");
        options_.queue_size = Options::getDoubleScalar(prhs[i]);
        break;

      case 3:
        if (!Options::isLogicalScalar(prhs[i]) && !Options::isDoubleScalar(prhs[i])) throw Exception("Publisher.advertise", "need logical latch as 4th argument (optional)");
        options_.latch = Options::getLogicalScalar(prhs[i]);
        break;

      default:
        throw ArgumentException("Publisher.advertise", "too many arguments");
    }
  }

  introspection_ = cpp_introspection::messageByDataType(options_.datatype);
  if (!introspection_) throw Exception("Publisher.advertise", "unknown datatype '" + options_.datatype + "'");
  options_.md5sum = introspection_->getMD5Sum();
  options_.message_definition = introspection_->getDefinition();
  options_.has_header = introspection_->hasHeader();

  *this = node_handle_.advertise(options_);
  return mxCreateLogicalScalar(*this);
}

void Publisher::publish(int nrhs, const mxArray *prhs[])
{
  if (nrhs < 1) throw ArgumentException("Publisher.publish", 1);
  if (!introspection_) throw Exception("Publisher.publish", "unknown message type");

  MessagePtr message;
  ros::SerializedMessage m;
  m.type_info = &(introspection_->getTypeId());
  Conversion conversion(introspection_);

  std::size_t count = conversion.numberOfInstances(prhs[0]);
  for(std::size_t i = 0; i < count; ++i) {
    message = conversion.fromMatlab(prhs[0], i);
    if (!message) throw Exception("Publisher.publish", "failed to parse message of type " + options_.datatype);

//    m.message = message->getConstInstance();
////    ROSMATLAB_PRINTF("Publishing on topic %s...", ros::Publisher::getTopic().c_str());
//    ros::TopicManager::instance()->publish(ros::Publisher::getTopic(), boost::bind(&serialize, message), m);
    ros::Publisher::publish(*message);
  }
}

mxArray *Publisher::getTopic() const
{
  return mxCreateString(ros::Publisher::getTopic().c_str());
}

mxArray *Publisher::getDataType() const
{
  if (!introspection_) return mxCreateString(0);
  return mxCreateString(introspection_->getDataType());
}

mxArray *Publisher::getMD5Sum() const
{
  if (!introspection_) return mxCreateString(0);
  return mxCreateString(introspection_->getMD5Sum());
}

mxArray *Publisher::getNumSubscribers() const
{
  return mxCreateDoubleScalar(ros::Publisher::getNumSubscribers());
}

mxArray *Publisher::isLatched() const
{
  throw Exception("ros::Publisher::isLatched is currently not implemented in roscpp (see https://github.com/ros/ros_comm/pull/1)");
//  return mxCreateLogicalScalar(ros::Publisher::isLatched());
  return mxCreateLogicalScalar(false);
}

} // namespace rosmatlab
