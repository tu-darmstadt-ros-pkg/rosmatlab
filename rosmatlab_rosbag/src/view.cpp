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
  if (nrhs > 0) addQuery(bag, nrhs, prhs);
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

mxArray *View::eof()
{
  return mxCreateLogicalScalar(eof_);
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
}

mxArray *View::getInternal(mxArray *target, std::size_t index, std::size_t size)
{
  if (!message_instance_ && (current_ != end())) {
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

  if (message_instance_) {
    target = Conversion(message_instance_).toMatlab(target, index, size);

  } else {
    target = mxCreateStructMatrix(0, 0, 0, 0);
  }

  return target;
}

void View::next(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  increment();
  if (nlhs > 0)
    get(nlhs, plhs, nrhs, prhs);
}

void View::increment() {
  message_instance_.reset();
  if (eof_) return;

  // reset current iterator
  if (current_ == end()) {
    current_ = begin();

  // or increment iterator
  } else {
    current_++;
  }

  if (current_ == end()) eof_ = true;
}

mxArray *View::getTime()
{
  if (current_ == end()) return mxCreateDoubleMatrix(0, 0, mxREAL);
  return mxCreateTime(current_->getTime());
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

mxArray *View::getBeginTime()
{
  return mxCreateTime(::rosbag::View::getBeginTime());
}

mxArray *View::getEndTime()
{
  return mxCreateTime(::rosbag::View::getEndTime());
}

Query::Query(int nrhs, const mxArray *prhs[])
  : start_time_(ros::TIME_MIN)
  , end_time_(ros::TIME_MAX)
{
  init(Options(nrhs, prhs, true));
}

Query::Query(const Options &options)
  : start_time_(ros::TIME_MIN)
  , end_time_(ros::TIME_MAX)
{
  init(options);
}

Query::Query(const std::string &topic)
  : start_time_(ros::TIME_MIN)
  , end_time_(ros::TIME_MAX)
{
  topics_.insert(topic);
}

Query::Query(const Query &other, const std::string &topic)
  : topics_(other.topics_)
  , datatypes_(other.datatypes_)
  , md5sums_(other.md5sums_)
  , start_time_(other.start_time_)
  , end_time_(other.end_time_)
{
  if (!topic.empty()) {
    topics_.clear();
    topics_.insert(topic);
  }
}

Query::~Query()
{
}

void Query::init(const Options &options)
{
  if (options.hasKey("start"))    start_time_ = ros::Time(options.getDouble("start"));
  if (options.hasKey("stop"))     end_time_   = ros::Time(options.getDouble("stop"));
  if (options.hasKey("topic"))    topics_.insert(options.getStrings("topic").begin(), options.getStrings("topic").end());
  if (options.hasKey("datatype")) datatypes_.insert(options.getStrings("datatype").begin(), options.getStrings("datatype").end());
  if (options.hasKey("md5sum"))   md5sums_.insert(options.getStrings("md5sum").begin(), options.getStrings("md5sum").end());

  // default option are topics
  if (options.hasKey(""))         topics_.insert(options.getStrings("").begin(), options.getStrings("").end());

  options.warnUnused();
}

bool Query::operator ()(const ConnectionInfo *info)
{
  if (topics_.size() > 0    && topics_.count(info->topic) == 0) return false;
  if (datatypes_.size() > 0 && datatypes_.count(info->datatype) == 0) return false;
  if (md5sums_.size() > 0   && md5sums_.count(info->md5sum) == 0) return false;
  return true;
}

mxArray *Query::toMatlab(const std::vector<boost::shared_ptr<Query> > &queries) {
  mxArray *result;
  static const char *fieldnames[] = { "Topic", "DataType", "MD5Sum", "StartTime", "EndTime" };
  result = mxCreateStructMatrix(queries.size(), 1, 5, fieldnames);

  for(mwIndex i = 0; i < queries.size(); ++i) {
    const Query &query = *queries[i];

    {
      mxArray *cell = mxCreateCellMatrix(1, query.topics_.size());
      mwIndex j = 0;
      for(Filter::const_iterator it = query.topics_.begin(); it != query.topics_.end(); ++it)
        mxSetCell(cell, j++, mxCreateString(it->c_str()));
      mxSetField(result, i, "Topic", cell);
    }

    {
      mxArray *cell = mxCreateCellMatrix(1, query.datatypes_.size());
      mwIndex j = 0;
      for(Filter::const_iterator it = query.datatypes_.begin(); it != query.datatypes_.end(); ++it)
        mxSetCell(cell, j++, mxCreateString(it->c_str()));
      mxSetField(result, i, "DataType", cell);
    }

    {
      mxArray *cell = mxCreateCellMatrix(1, query.md5sums_.size());
      mwIndex j = 0;
      for(Filter::const_iterator it = query.md5sums_.begin(); it != query.md5sums_.end(); ++it)
        mxSetCell(cell, j++, mxCreateString(it->c_str()));
      mxSetField(result, i, "MD5Sum", cell);
    }

    mxSetField(result, i, "StartTime", mxCreateTime(query.getStartTime()));
    mxSetField(result, i, "EndTime",   mxCreateTime(query.getEndTime()));
  }

  return result;
}

} // namespace rosbag
} // namespace rosmatlab
