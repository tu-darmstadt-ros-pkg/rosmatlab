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

#include <rosmatlab/connection_header.h>
#include <rosmatlab/options.h>
#include <boost/lexical_cast.hpp>

namespace rosmatlab {

ConnectionHeader::ConnectionHeader()
{
}

ConnectionHeader::ConnectionHeader(const mxArray *array)
{
  fromMatlab(array);
}

ConnectionHeader::ConnectionHeader(const boost::shared_ptr<ros::M_string> &connection_header)
  : data_(connection_header)
{
}

ConnectionHeader::~ConnectionHeader()
{
}

bool ConnectionHeader::fromMatlab(const mxArray *array)
{
  data_.reset();
  if (!mxIsStruct(array) && !mxGetNumberOfElements(array) == 1) return false;

  data_.reset(new ros::M_string);
  int numberOfFields = mxGetNumberOfFields(array);
  for(int i = 0; i < numberOfFields; i++) {
    std::string key = mxGetFieldNameByNumber(array, i);
    const mxArray *value = mxGetFieldByNumber(array, 0, i);
    if (Options::isString(value)) {
      data_->insert(ros::StringPair(key, Options::getString(value)));
    } else if (Options::isDoubleScalar(value)) {
      data_->insert(ros::StringPair(key, boost::lexical_cast<std::string>(Options::getDoubleScalar(value))));
    } else if (Options::isIntegerScalar(value)) {
      data_->insert(ros::StringPair(key, boost::lexical_cast<std::string>(Options::getIntegerScalar(value))));
    } else if (Options::isLogicalScalar(value)) {
      data_->insert(ros::StringPair(key, boost::lexical_cast<std::string>(Options::getLogicalScalar(value))));
    }
  }

  return true;
}

mxArray *ConnectionHeader::toMatlab() const {
  mxArray *header = mxCreateStructMatrix(1, 1, 0, 0);
  if (!data_) return header;

  for(ros::M_string::iterator it = data_->begin(); it != data_->end(); ++it) {
    mxAddField(header, it->first.c_str());
    mxSetField(header, 0, it->first.c_str(), mxCreateString(it->second.c_str()));
  }

  return header;
}

}
