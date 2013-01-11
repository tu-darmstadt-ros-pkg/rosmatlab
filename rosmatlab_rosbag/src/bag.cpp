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

#include <rosmatlab/rosbag/bag.h>
#include <rosmatlab/conversion.h>
#include <rosmatlab/connection_header.h>
#include <rosmatlab/log.h>

#include <rosmatlab/rosbag/view.h>

#include <introspection/message.h>
#include <boost/algorithm/string/replace.hpp>

namespace rosmatlab {
namespace rosbag {

template <> const char *Object<Bag>::class_name_ = "rosbag.Bag";

Bag::Bag()
  : Object<Bag>(this)
{
}

Bag::Bag(int nrhs, const mxArray *prhs[])
  : Object<Bag>(this)
{
  if (nrhs > 0) open(nrhs, prhs);
}

Bag::~Bag() {
  close();
}

void Bag::open(int nrhs, const mxArray *prhs[])
{
  if (nrhs < 1) throw ArgumentException("Bag.open", 1);
  std::string filename = Options::getString(prhs[0]);
  uint32_t mode = ::rosbag::bagmode::Read;

  if (nrhs >= 2) {
    mode = Options::getIntegerScalar(prhs[1]);
    if (mode > 7) throw Exception("Bag.open", "Invalid value for mode");
  }

  try {
    ::rosbag::Bag::open(filename, mode);
  } catch(std::runtime_error &e) {
    throw Exception(e);
  }
}

void Bag::close()
{
  ::rosbag::Bag::close();
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

void Bag::get(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  View view(*this, nrhs, prhs);
  std::vector<const ConnectionInfo *> connections = static_cast< ::rosbag::View& >(view).getConnections();

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
  view.increment(); // go to first entry
  for( ; view.valid(); view.increment()) {
    if (!topics.count(view->getTopic())) continue;
    FieldInfo &field = topics[view->getTopic()];
    mxArray *target = mxGetFieldByNumber(data, 0, field.fieldnum);

    if (!target) {
      // find out how many messages are in the bag file
      Query only_this_topic(*(view.queries_.front()), view->getTopic());
      field.size = ::rosbag::View(*this, only_this_topic, only_this_topic.getStartTime(), only_this_topic.getEndTime()).size();
      // ROSMATLAB_PRINTF("Field %s has %u entries.", field.name.c_str(), field.size);
    }

    assert(field.index < field.size);
    // ROSMATLAB_PRINTF("Converting entry %u/%u of field %s", field.index, field.size, field.name.c_str());
    target = view.getInternal(target, field.index++, field.size);
    mxSetFieldByNumber(data, 0, field.fieldnum, target);
  }

  // return result
  plhs[0] = data;
}

mxArray *Bag::getFileName() const
{
  return mxCreateString(::rosbag::Bag::getFileName().c_str());
}

mxArray *Bag::getMode() const
{
  mxArray *result = mxCreateNumericMatrix(1, 1, mxUINT8_CLASS, mxREAL);
  *static_cast<uint8_T *>(mxGetData(result)) = static_cast<uint8_T>(::rosbag::Bag::getMode());
  return result;
}

mxArray *Bag::getMajorVersion() const
{
  mxArray *result = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
  *static_cast<uint32_T *>(mxGetData(result)) = static_cast<uint32_T>(::rosbag::Bag::getMajorVersion());
  return result;
}

mxArray *Bag::getMinorVersion() const
{
  mxArray *result = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
  *static_cast<uint32_T *>(mxGetData(result)) = static_cast<uint32_T>(::rosbag::Bag::getMinorVersion());
  return result;
}

mxArray *Bag::getSize() const
{
  mxArray *result = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
  *static_cast<uint64_T *>(mxGetData(result)) = static_cast<uint64_T>(::rosbag::Bag::getSize());
  return result;
}

void Bag::setCompression(int nrhs, const mxArray *prhs[])
{
  if (nrhs < 1) throw ArgumentException("Bag.setCompression", 1);
  int value = Options::getIntegerScalar(prhs[0]);
  if (value < 0 || value > ::rosbag::compression::BZ2) throw Exception("Bag.setCompression", "Invalid value for property Compression");
  ::rosbag::compression::CompressionType compression = static_cast< ::rosbag::compression::CompressionType >(Options::getIntegerScalar(prhs[0]));
  ::rosbag::Bag::setCompression(compression);
}

mxArray *Bag::getCompression() const
{
  mxArray *result = mxCreateNumericMatrix(1, 1, mxUINT8_CLASS, mxREAL);
  *static_cast<uint8_T *>(mxGetData(result)) = static_cast<uint8_T>(::rosbag::Bag::getCompression());
  return result;
}

void Bag::setChunkThreshold(int nrhs, const mxArray *prhs[])
{
  if (nrhs < 1) throw ArgumentException("Bag.setChunkThreshold", 1);
  uint32_t chunkThreshold = static_cast<uint32_t>(Options::getIntegerScalar(prhs[0]));
  ::rosbag::Bag::setChunkThreshold(chunkThreshold);
}

mxArray *Bag::getChunkThreshold() const
{
  mxArray *result = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
  *static_cast<uint32_T *>(mxGetData(result)) = static_cast<uint32_T>(::rosbag::Bag::getChunkThreshold());
  return result;
}

void Bag::write(int nrhs, const mxArray *prhs[])
{
  if (nrhs < 3) throw ArgumentException("Bag.write", 3);

  // parse arguments
  std::string topic = Options::getString(prhs[0]);
  std::string datatype = Options::getString(prhs[1]);
  const mxArray *data = prhs[2];
  ConnectionHeader connection_header;
  ros::Time timestamp;
  for(int i = 3; i < nrhs; ++i) {
    if (mxIsStruct(prhs[i])) {
      if (!connection_header.fromMatlab(prhs[i])) throw Exception("Bag.write", "failed to parse connection header");
      continue;
    }

    if (Options::isDoubleScalar(prhs[i])) {
      timestamp = ros::Time(Options::getDoubleScalar(prhs[i]));
      continue;
    }
  }

  // set timestamp to current time if no timestamp was given
  if (timestamp.isZero()) timestamp = ros::Time::now();

  // introspect message
  MessagePtr introspection = cpp_introspection::messageByDataType(datatype);
  if (!introspection) throw Exception("Bag.write", "unknown datatype '" + datatype + "'");
  Conversion conversion(introspection);

  // ... and finally write data to the bag
  std::size_t count = conversion.numberOfInstances(data);
  for(std::size_t i = 0; i < count; ++i) {
    MessagePtr message = Conversion(introspection).fromMatlab(data, i);
    if (!message) throw Exception("Bag.write", "failed to parse message of type " + datatype);

    ::rosbag::Bag::write(topic, timestamp, *message, connection_header);
  }
}

}
}
