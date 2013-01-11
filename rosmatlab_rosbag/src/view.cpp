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

#include <rosmatlab/rosbag/view.h>
#include <rosmatlab/options.h>
#include <rosmatlab/connection_header.h>
#include <rosmatlab/conversion.h>
#include <rosmatlab/log.h>

#include <rosmatlab/rosbag/bag.h>

#include <ros/forwards.h>
#include <introspection/message.h>

namespace rosmatlab {
namespace rosbag {

using ::rosbag::ConnectionInfo;

class View;

class Query
{
public:
  Query(const Options& options);
  bool operator()(const ConnectionInfo *info);

  const ros::Time &getStartTime() const { return start_time_; }
  const ros::Time &getStopTime() const { return stop_time_; }

  static mxArray *toMatlab(const std::vector<boost::shared_ptr<Query> >&);

private:
  ros::M_string filter_;
  ros::Time start_time_;
  ros::Time stop_time_;
};
typedef boost::shared_ptr<Query> QueryPtr;

template <> const char *Object<View>::class_name_ = "rosbag.View";

View::View(int nrhs, const mxArray *prhs[])
  : Object<View>(this)
{
  reset();
  if (nrhs > 0) addQuery(nrhs, prhs);
}

View::~View()
{
}

mxArray *View::getSize()
{
  return mxCreateDoubleScalar(size());
}

void View::addQuery(int nrhs, const mxArray *prhs[])
{
  if (nrhs < 1) throw ArgumentException("View.addQuery", 1);

  const Bag *bag = getObject<Bag>(prhs[0]);
  if (!bag) throw Exception("View.addQuery", "first argument is not a valid bag handle");

  Options options(nrhs - 1, prhs + 1, true);
  QueryPtr query(new Query(options));

  ::rosbag::View::addQuery(*bag, *query, query->getStartTime(), query->getStopTime());
  queries_.push_back(query);
}

void View::reset()
{
  current_ = end();
  eof_ = false;
}

mxArray *View::eof()
{
  return mxCreateLogicalScalar(eof_);
}

mxArray *View::get(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (message_instance_) {
    plhs[0] = Conversion(message_instance_).toMatlab();
  } else {
    plhs[0] = mxCreateStructMatrix(0, 0, 0, 0);
  }

  if (nlhs > 1) plhs[1] = getTopic();
  if (nlhs > 2) plhs[2] = getDataType();
  return plhs[0];
}

mxArray *View::next(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  message_instance_.reset();

  if (!eof_) {
    // reset current iterator
    if (current_ == end()) {
      current_ = begin();

    // or increment iterator
    } else {
      current_++;
    }

    if (current_ != end()) {
      MessagePtr introspection = messageByMD5Sum(current_->getMD5Sum());
      if (introspection) {
        // copy the serialized message to the read buffer (ugly)
        std::size_t size = current_->size();
        if (read_buffer_.size() < size) read_buffer_.resize(size);
        ros::serialization::OStream ostream(read_buffer_.data(), read_buffer_.size());
        current_->write(ostream);

        // deserialize message
        ros::serialization::IStream istream(read_buffer_.data(), size);
        VoidPtr msg = introspection->deserialize(istream);
        if (!msg) ROSMATLAB_WARN("deserialization of a message of type %s failed", current_->getDataType().c_str());

        message_instance_ = introspection->introspect(msg);
      } else {
        mexPrintf(ROSMATLAB_PRINTF_PREFIX "Unknown data type '%s' in bag file\n", current_->getDataType().c_str());
        // throw UnknownDataTypeException(current_->getDataType());
      }
    } else
      eof_ = true;
  }

  return get(nlhs, plhs, nrhs, prhs);
}

mxArray *View::getTime()
{
  if (current_ == end()) return mxCreateDoubleMatrix(0, 0, mxREAL);
  return mxCreateDoubleScalar(current_->getTime().toSec());
}

mxArray *View::getTopic()
{
  if (current_ == end()) return mxCreateString(0);
  return mxCreateString(current_->getTopic().c_str());
}

mxArray *View::getDataType()
{
  if (current_ == end()) return mxCreateString(0);
  return mxCreateString(current_->getDataType().c_str());
}

mxArray *View::getMD5Sum()
{
  if (current_ == end()) return mxCreateString(0);
  return mxCreateString(current_->getMD5Sum().c_str());
}

mxArray *View::getMessageDefinition()
{
  if (current_ == end()) return mxCreateString(0);
  return mxCreateString(current_->getMessageDefinition().c_str());
}

mxArray *View::getConnectionHeader()
{
  if (current_ == end()) return mxCreateStructMatrix(0, 0, 0, 0);
  return ConnectionHeader(current_->getConnectionHeader()).toMatlab();
}

mxArray *View::getCallerId()
{
  if (current_ == end()) return mxCreateString(0);
  return mxCreateString(current_->getCallerId().c_str());
}

mxArray *View::isLatching()
{
  if (current_ == end()) return mxCreateLogicalMatrix(0, 0);
  return mxCreateLogicalScalar(current_->isLatching());
}

mxArray *View::getQueries()
{
  return Query::toMatlab(queries_);
}

mxArray *View::getConnections()
{
  std::vector<const ConnectionInfo *> connections = ::rosbag::View::getConnections();

  mxArray *result;
  static const char *fieldnames[] = { "Id", "Topic", "DataType", "MD5Sum" };
  result = mxCreateStructMatrix(connections.size(), 1, 4, fieldnames);

  for(mwIndex i = 0; i < connections.size(); ++i) {
    mxSetField(result, i, "Id",       mxCreateDoubleScalar(connections[i]->id));
    mxSetField(result, i, "Topic",    mxCreateString(connections[i]->topic.c_str()));
    mxSetField(result, i, "DataType", mxCreateString(connections[i]->datatype.c_str()));
    mxSetField(result, i, "MD5Sum",   mxCreateString(connections[i]->md5sum.c_str()));
  }

  return result;
}

Query::Query(const Options &options)
  : filter_(options.strings())
  , start_time_(ros::TIME_MIN)
  , stop_time_(ros::TIME_MAX)
{
  if (options.hasKey("start")) start_time_ = ros::Time(options.getDouble("start"));
  if (options.hasKey("stop"))  stop_time_  = ros::Time(options.getDouble("stop"));
  options.getString("topic"); // mark as used
  options.getString("datatype"); // mark as used
  options.getString("md5sum"); // mark as used
  options.warnUnused();
}

bool Query::operator ()(const ConnectionInfo *info)
{
  if (filter_.count("topic")    && info->topic    != filter_["topic"])    return false;
  if (filter_.count("datatype") && info->datatype != filter_["datatype"]) return false;
  if (filter_.count("md5sum")   && info->md5sum   != filter_["md5sum"])   return false;
  return true;
}

mxArray *Query::toMatlab(const std::vector<boost::shared_ptr<Query> > &queries) {
  mxArray *result;
  static const char *fieldnames[] = { "Topic", "DataType", "MD5Sum", "StartTime", "StopTime" };
  result = mxCreateStructMatrix(queries.size(), 1, 5, fieldnames);

  for(mwIndex i = 0; i < queries.size(); ++i) {
    const Query &query = *queries[i];
    if (query.filter_.count("topic"))    mxSetField(result, i, "Topic", mxCreateString(query.filter_.at("topic").c_str()));
    if (query.filter_.count("datatype")) mxSetField(result, i, "Topic", mxCreateString(query.filter_.at("topic").c_str()));
    if (query.filter_.count("topic"))    mxSetField(result, i, "Topic", mxCreateString(query.filter_.at("topic").c_str()));
  }

  return result;
}

} // namespace rosbag
} // namespace rosmatlab
