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

#ifndef ROSMATLAB_ROSBAG_QUERY_H
#define ROSMATLAB_ROSBAG_QUERY_H

#include <rosmatlab/options.h>
#include <rosbag/query.h>
#include <introspection/forwards.h>

namespace rosmatlab {
namespace rosbag {

using ::rosbag::ConnectionInfo;

class Query
{
public:
  Query();
  Query(int nrhs, const mxArray *prhs[]);
  Query(const Options& options);
  Query(const std::string& topic);
  virtual ~Query();

  virtual void init(const Options& options);
  virtual bool operator()(const ConnectionInfo *info);

  ros::Time const& getStartTime() const;
  ros::Time const& getEndTime()   const;

  static mxArray *toMatlab(const std::vector<boost::shared_ptr<Query> >&);

private:
  typedef std::set<std::string> Filter;
  Filter topics_;
  Filter datatypes_;
  Filter md5sums_;

  ros::Time start_time_;
  ros::Time end_time_;
};
typedef boost::shared_ptr<Query> QueryPtr;

// used internally
class TopicFilterQuery
{
public:
  TopicFilterQuery(::rosbag::Query& base, const std::string& topic);
  virtual ~TopicFilterQuery();

  virtual bool operator()(const ConnectionInfo *info);

private:
  ::rosbag::Query& base_;
  std::string topic_;
};

} // namespace rosbag
} // namespace rosmatlab

#endif // ROSMATLAB_ROSBAG_QUERY_H
