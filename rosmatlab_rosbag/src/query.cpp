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

#include <rosmatlab/rosbag/query.h>

#include <rosmatlab/options.h>
#include <rosmatlab/connection_header.h>
#include <rosmatlab/conversion.h>
#include <rosmatlab/log.h>

#include <ros/forwards.h>
#include <rosbag/structures.h>

#include <introspection/message.h>

#include <boost/algorithm/string/replace.hpp>


namespace rosmatlab {
namespace rosbag {

Query::Query()
  : start_time_(ros::TIME_MIN)
  , end_time_(ros::TIME_MAX)
{
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

  options.throwOnUnused();
}

bool Query::operator()(const ConnectionInfo *info)
{
  if (topics_.size() > 0    && topics_.count(info->topic) == 0) return false;
  if (datatypes_.size() > 0 && datatypes_.count(info->datatype) == 0) return false;
  if (md5sums_.size() > 0   && md5sums_.count(info->md5sum) == 0) return false;
  return true;
}

ros::Time const& Query::getStartTime() const { return start_time_; }
ros::Time const& Query::getEndTime()   const { return end_time_;   }

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

TopicFilterQuery::TopicFilterQuery(::rosbag::Query& base, const std::string& topic)
  : base_(base)
  , topic_(topic)
{
}

TopicFilterQuery::~TopicFilterQuery()
{
}

bool TopicFilterQuery::operator()(const ConnectionInfo *info) {
  return (base_.getQuery()(info)) && (info->topic == topic_);
}


} // namespace rosbag
} // namespace rosmatlab

