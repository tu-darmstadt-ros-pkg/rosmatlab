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

#include <rosmatlab/param.h>
#include <rosmatlab/exception.h>

#include <rosmatlab/options.h>
#include <ros/param.h>

#include <mex.h>

namespace rosmatlab {
namespace param {

using namespace XmlRpc;

mxArray *toMatlab(XmlRpcValue& value) {
  mxArray *array = 0;

  switch(value.getType()) {
    case XmlRpcValue::TypeBoolean:
      array = mxCreateLogicalScalar(static_cast<bool>(value));
      break;

    case XmlRpcValue::TypeInt:
      array = mxCreateDoubleScalar(static_cast<int>(value));
      break;

    case XmlRpcValue::TypeDouble:
      array = mxCreateDoubleScalar(static_cast<double>(value));
      break;

    case XmlRpcValue::TypeString:
      array = mxCreateString(static_cast<std::string>(value).c_str());
      break;

    case XmlRpcValue::TypeDateTime:
      // not supported
      break;

    case XmlRpcValue::TypeBase64:
      // not supported
      break;

    case XmlRpcValue::TypeArray:
      {
        mxArray *a = mxCreateCellMatrix(value.size(), 1);
        for(int i = 0; i < value.size(); ++i) {
          mxSetCell(a, i, toMatlab(value[i]));
        }
        array = a;
      }
      break;

    case XmlRpcValue::TypeStruct:
      {
        std::vector<const char *> fieldnames;
        XmlRpcValue::iterator it;
        for(it = value.begin(); it != value.end(); ++it) {
          fieldnames.push_back(it->first.c_str());
        }
        mxArray *s = mxCreateStructMatrix(1, 1, fieldnames.size(), fieldnames.data());
        for(it = value.begin(); it != value.end(); ++it) {
          mxSetField(s, 0, it->first.c_str(), toMatlab(it->second));
        }
        array = s;
      }
      break;

    default:
      break;
  }

  return array;
}

void fromMatlabBool(const mxArray *array, XmlRpcValue& value) {
  size_t n = mxGetNumberOfElements(array);
  mxLogical *data = mxGetLogicals(array);

  if (n == 1) {
    value = XmlRpcValue(data[0]);
  } else {
    value.setSize(n);
    for(int i = 0; i < n; ++i) value[i] = data[i];
  }
}

void fromMatlabString(const mxArray *array, XmlRpcValue& value) {
  size_t n = mxGetNumberOfElements(array);
  std::vector<char> buffer(n + 1);
  if (mxGetString(array, buffer.data(), buffer.size()) == 0) {
    value = buffer.data();
  }
}

template <typename From, typename To>
void fromMatlabNumeric(const mxArray *array, XmlRpcValue& value) {
  size_t n = mxGetNumberOfElements(array);
  const From *data = static_cast<const From *>(mxGetData(array));

  if (n == 1) {
    value = static_cast<To>(*data);
  } else {
    value.setSize(n);
    for(int i = 0; i < n; ++i) value[i] = static_cast<To>(data[i]);
  }
}

bool fromMatlab(const mxArray *array, XmlRpcValue& value) {
  // invalidate value
  value.clear();

  switch(mxGetClassID(array)) {
    case mxCELL_CLASS:
      // not supported
      break;

    case mxSTRUCT_CLASS:
      // not supported
      break;

    case mxLOGICAL_CLASS:
      fromMatlabBool(array, value);
      break;

    case mxCHAR_CLASS:
      fromMatlabString(array, value);
      break;

    case mxDOUBLE_CLASS:
      fromMatlabNumeric<double, double>(array, value);
      break;

    case mxSINGLE_CLASS:
      fromMatlabNumeric<float, double>(array, value);
      break;

    case mxINT8_CLASS:
      fromMatlabNumeric<int8_t, int>(array, value);
      break;

    case mxUINT8_CLASS:
      fromMatlabNumeric<uint8_t, int>(array, value);
      break;

    case mxINT16_CLASS:
      fromMatlabNumeric<int16_t, int>(array, value);
      break;

    case mxUINT16_CLASS:
      fromMatlabNumeric<uint16_t, int>(array, value);
      break;

    case mxINT32_CLASS:
      fromMatlabNumeric<int32_t, int>(array, value);
      break;

    case mxUINT32_CLASS:
      fromMatlabNumeric<uint32_t, int>(array, value);
      break;

    case mxINT64_CLASS:
      fromMatlabNumeric<int64_t, int>(array, value);
      break;

    case mxUINT64_CLASS:
      fromMatlabNumeric<uint64_t, int>(array, value);
      break;

    case mxFUNCTION_CLASS:
      // not supported
      break;

    default:
      break;
  }

  return value.valid();
}

void get(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  std::string key;
  XmlRpcValue value;

  if (nrhs > 0) key = Options::getString(prhs[0]);

  // get parameter
  try {
    if (!ros::param::get(key, value)) {
      plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
      return;
    }
  } catch (ros::InvalidNameException& e) {
    throw Exception(e.what());
  }

  // convert parameter value to Matlab
  plhs[0] = toMatlab(value);
  if (!plhs[0]) {
    throw Exception("Could not convert the parameter value of " + key + " to Matlab (unknown type)");
    plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
    return;
  }

  return;
}

void set(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs < 2) {
    throw Exception("ros.param.set needs two input arguments");
  }

  std::string key = Options::getString(prhs[0]);
  XmlRpcValue value;

  // convert parameter value from Matlab
  if (!fromMatlab(prhs[1], value)) {
    throw Exception("Could not convert the parameter value for " + key + " to a XmlRpcValue");
    return;
  }

  // set parameter
  try {
    ros::param::set(key, value);
  } catch (ros::InvalidNameException& e) {
    throw Exception(e.what());
  }
}

void del(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs < 1) {
    throw Exception("ros.param.delete needs an input argument");
  }

  std::string key = Options::getString(prhs[0]);
  if (nlhs > 0) plhs[0] = mxCreateLogicalScalar(ros::param::del(key));
}

void has(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs < 1) {
    throw Exception("ros.param.has needs an input argument");
  }

  std::string key = Options::getString(prhs[0]);
  plhs[0] = mxCreateLogicalScalar(ros::param::has(key));
}

} // namespace param
} // namespace rosmatlab
