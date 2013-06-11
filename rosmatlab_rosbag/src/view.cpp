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
#include <rosmatlab/rosbag/bag.h>
#include <rosmatlab/rosbag/query.h>

#include <rosmatlab/options.h>
#include <rosmatlab/connection_header.h>
#include <rosmatlab/conversion.h>
#include <rosmatlab/log.h>

#include <ros/forwards.h>
#include <introspection/message.h>

#include <boost/algorithm/string/replace.hpp>


namespace rosmatlab {
namespace rosbag {

template <> const char *Object<View>::class_name_ = "rosbag.View";

View::View(int nrhs, const mxArray *prhs[])
  : Object<View>(this)
{
  reset();
  if (nrhs > 0) addQuery(nrhs, prhs);
}

View::View(const Bag& bag, int nrhs, const mxArray *prhs[])
  : Object<View>(this)
{
  reset();
  addQuery(bag, nrhs, prhs);
}

View::~View()
{
}

mxArray *View::getSize()
{
  return mxCreateDoubleScalar(::rosbag::View::size());
}

void View::addQuery(int nrhs, const mxArray *prhs[])
{
  if (nrhs < 1) throw ArgumentException("View.addQuery", 1);

  const Bag *bag = getObject<Bag>(prhs[0]);
  if (!bag) throw Exception("View.addQuery", "first argument is not a valid bag handle");

  addQuery(*bag, nrhs - 1, prhs + 1);
}

void View::addQuery(const Bag& bag, int nrhs, const mxArray *prhs[])
{
  QueryPtr query(new Query(nrhs, prhs));
  ::rosbag::View::addQuery(bag, *query, query->getStartTime(), query->getEndTime());
  queries_.push_back(query);
}

void View::reset()
{
  current_ = end();
  eof_ = false;
}

bool View::start()
{
  current_ = begin();
  eof_ = false;
  return valid();
}

void View::increment() {
  message_instance_.reset();
  if (eof_) return;

  // reset current iterator
  if (!valid()) {
    start();

  // or increment iterator
  } else {
    current_++;
  }

  if (!valid()) eof_ = true;
}

bool View::eof()
{
  return eof_;
}

bool View::valid()
{
  return current_ != end();
}

View::iterator& View::operator*()
{
  return current_;
}

MessageInstance* View::operator->()
{
  return &(*current_);
}

void View::get(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  plhs[0] = getInternal(plhs[0]);
  if (nlhs > 1) plhs[1] = getTopic();
  if (nlhs > 2) plhs[2] = getDataType();
  if (nlhs > 3) plhs[3] = getConnectionHeader();
  if (nlhs > 4) plhs[4] = getTime();
}

void View::next(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  increment();
  if (nlhs > 0) get(nlhs, plhs, nrhs, prhs);
}

mxArray *View::getInternal(mxArray *target, std::size_t index, std::size_t size)
{
   // go to the first entry if the current iterator is not valid
  if (!valid()) increment();

  // introspect message
  if (valid() && !message_instance_) {
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
      ROSMATLAB_PRINTF("Unknown data type '%s' in bag file", current_->getDataType().c_str());
      // throw UnknownDataTypeException(current_->getDataType());
    }
  }

  // convert message instance to Matlab
  if (message_instance_) {
    target = Conversion(message_instance_).toMatlab(target, index, size);
  } else {
    target = mxCreateStructMatrix(0, 0, 0, 0);
  }

  return target;
}

namespace {
  struct FieldInfo {
    std::string topic;
    std::string name;
    int fieldnum;
    std::size_t index;
    std::size_t size;
  };
}

void View::data(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  std::vector<const ConnectionInfo *> connections = ::rosbag::View::getConnections();

  // extract topic information from Connections
  std::map<std::string, FieldInfo> topics;
  std::vector<const char *> fieldnames;
  fieldnames.reserve(connections.size());

  for(std::vector<const ConnectionInfo *>::iterator it = connections.begin(); it != connections.end(); ++it) {
    const ConnectionInfo *c = *it;
    FieldInfo field;
    field.topic = c->topic;
    field.name = c->topic;
    boost::algorithm::replace_all(field.name, "/", "_");
    if (field.name.at(0) == '_') field.name = field.name.substr(1);
    fieldnames.push_back(field.name.c_str());
    field.fieldnum = topics.size();
    field.index = 0;
    field.size = 0;
    topics[c->topic] = field;
  }

  // create result struct
  mxArray *data = mxCreateStructMatrix(1, 1, fieldnames.size(), fieldnames.data());

  // iterate through View
  for(start(); valid(); increment()) {
    if (!topics.count(current_->getTopic())) continue;
    FieldInfo &field = topics[current_->getTopic()];
    mxArray *target = mxGetFieldByNumber(data, 0, field.fieldnum);

    if (!target) {
      // find out how many messages are in the bag file by iterating over all (rosbag::Queries) and adding a new TopicFilterQuery object for each
      ::rosbag::View only_this_topic(reduce_overlap_);
      for(std::vector< ::rosbag::BagQuery* >::iterator query = ::rosbag::View::queries_.begin(); query != ::rosbag::View::queries_.end(); ++query) {
        only_this_topic.addQuery(*((*query)->bag), TopicFilterQuery((*query)->query, current_->getTopic()));
      }
      field.size = only_this_topic.size();
      // ROSMATLAB_PRINTF("Field %s has %u entries.", field.name.c_str(), field.size);
    }

    assert(field.index < field.size);
    // ROSMATLAB_PRINTF("Converting entry %u/%u of field %s", field.index, field.size, field.name.c_str());
    target = getInternal(target, field.index++, field.size);
//    if (!target) target = mxCreateDoubleScalar(field.size); // debugging only

    mxSetFieldByNumber(data, 0, field.fieldnum, target);
  }

  // return result
  plhs[0] = data;
}

mxArray *View::getTime()
{
  if (!valid()) return mxCreateEmpty();
  return mxCreateTime(current_->getTime());
}

mxArray *View::getTopic()
{
  if (!valid()) return mxCreateEmpty();
  return mxCreateString(current_->getTopic().c_str());
}

mxArray *View::getDataType()
{
  if (!valid()) return mxCreateEmpty();
  return mxCreateString(current_->getDataType().c_str());
}

mxArray *View::getMD5Sum()
{
  if (!valid()) return mxCreateEmpty();
  return mxCreateString(current_->getMD5Sum().c_str());
}

mxArray *View::getMessageDefinition()
{
  if (!valid()) return mxCreateEmpty();
  return mxCreateString(current_->getMessageDefinition().c_str());
}

mxArray *View::getConnectionHeader()
{
  if (!valid()) return mxCreateStructMatrix(0, 0, 0, 0);
  return ConnectionHeader(current_->getConnectionHeader()).toMatlab();
}

mxArray *View::getCallerId()
{
  if (!valid()) return mxCreateEmpty();
  return mxCreateString(current_->getCallerId().c_str());
}

mxArray *View::isLatching()
{
  if (!valid()) return mxCreateLogicalMatrix(0, 0);
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

mxArray *View::getBeginTime()
{
  return mxCreateTime(::rosbag::View::getBeginTime());
}

mxArray *View::getEndTime()
{
  return mxCreateTime(::rosbag::View::getEndTime());
}

} // namespace rosbag
} // namespace rosmatlab
