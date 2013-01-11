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

#ifndef ROSMATLAB_ROSBAG_VIEW_H
#define ROSMATLAB_ROSBAG_VIEW_H

#include <rosbag/view.h>
#include <rosmatlab/object.h>

#include <introspection/forwards.h>

namespace rosmatlab {
namespace rosbag {

using ::rosbag::MessageInstance;
using ::rosbag::ConnectionInfo;

class Bag;
class View;

class Query
{
public:
  Query(int nrhs, const mxArray *prhs[]);
  Query(const Options& options);
  Query(const std::string& topic);
  Query(const Query& other, const std::string& topic);
  ~Query();

  bool operator()(const ConnectionInfo *info);

  void init(const Options& options);

  const ros::Time &getStartTime() const { return start_time_; }
  const ros::Time &getEndTime() const { return end_time_; }

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

class View : public ::rosbag::View, public Object<View> {
public:
  friend class Bag;

  using ::rosbag::View::iterator;
  using ::rosbag::View::const_iterator;

  View(int nrhs, const mxArray *prhs[]);
  View(const Bag& bag, int nrhs, const mxArray *prhs[]);
  virtual ~View();

  using ::rosbag::View::begin;
  using ::rosbag::View::end;
  using ::rosbag::View::size;

  mxArray *getSize();

  void addQuery(int nrhs, const mxArray *prhs[]);
  void addQuery(const Bag& bag, int nrhs, const mxArray *prhs[]);

  void reset();
  void increment();
  bool valid();

  mxArray *eof();

  void get(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
  void next(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

  mxArray *getTime();
  mxArray *getTopic();
  mxArray *getDataType();
  mxArray *getMD5Sum();
  mxArray *getMessageDefinition();
  mxArray *getConnectionHeader();
  mxArray *getCallerId();
  mxArray *isLatching();

  mxArray *getQueries();
  mxArray *getConnections();
  mxArray *getBeginTime();
  mxArray *getEndTime();

private:
  iterator& operator*();
  MessageInstance* operator->();
  mxArray *getInternal(mxArray *target, std::size_t index = 0, std::size_t size = 0);

private:
  std::vector<boost::shared_ptr<Query> > queries_;
  std::vector<uint8_t> read_buffer_;

  cpp_introspection::MessagePtr message_instance_;
  iterator current_;
  bool eof_;
};

} // namespace rosbag
} // namespace rosmatlab

#endif // ROSMATLAB_ROSBAG_VIEW_H
