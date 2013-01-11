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

#include <rosmatlab/options.h>
#include <limits>

#include <mex.h>

#include <boost/algorithm/string.hpp>

namespace rosmatlab {

template <typename T>
static T getScalar(const mxArray *value)
{
  if (value && mxGetNumberOfElements(value) == 1) {
    switch(mxGetClassID(value)) {
      case mxSINGLE_CLASS:
        return *static_cast<float *>(mxGetData(value));
      case mxDOUBLE_CLASS:
        return *static_cast<double *>(mxGetData(value));
      case mxINT8_CLASS:
        return *static_cast<int8_T *>(mxGetData(value));
      case mxUINT8_CLASS:
        return *static_cast<uint8_T *>(mxGetData(value));
      case mxINT16_CLASS:
        return *static_cast<int16_T *>(mxGetData(value));
      case mxUINT16_CLASS:
        return *static_cast<uint16_T *>(mxGetData(value));
      case mxINT32_CLASS:
        return *static_cast<int32_T *>(mxGetData(value));
      case mxUINT32_CLASS:
        return *static_cast<uint32_T *>(mxGetData(value));
      case mxINT64_CLASS:
        return *static_cast<int64_T *>(mxGetData(value));
      case mxUINT64_CLASS:
        return *static_cast<uint64_T *>(mxGetData(value));

      default: break;
    }
  }

  return std::numeric_limits<T>::quiet_NaN();
}

bool Options::isScalar(const mxArray *value)
{
  return value && mxGetNumberOfElements(value) == 1;
}

bool Options::isString(const mxArray *value)
{
  return value && mxIsChar(value);
}

std::string Options::getString(const mxArray *value)
{
  if (!isString(value)) return std::string();
  std::size_t len = mxGetNumberOfElements(value);
  char temp[len + 1];
  mxGetString(value, temp, sizeof(temp));
  return std::string(temp);
}

bool Options::isDoubleScalar(const mxArray *value)
{
  if (!isScalar(value)) return false;

  switch(mxGetClassID(value)) {
    case mxSINGLE_CLASS:
    case mxDOUBLE_CLASS:
      return true;
    default: break;
  }

  return false;
}

double Options::getDoubleScalar(const mxArray *value)
{
  return getScalar<double>(value);
}

bool Options::isIntegerScalar(const mxArray *value)
{
  if (!isScalar(value)) return false;

  switch(mxGetClassID(value)) {
    case mxINT8_CLASS:
    case mxUINT8_CLASS:
    case mxINT16_CLASS:
    case mxUINT16_CLASS:
    case mxINT32_CLASS:
    case mxUINT32_CLASS:
    case mxINT64_CLASS:
    case mxUINT64_CLASS:
      return true;

    default: break;
  }

  return false;
}

int Options::getIntegerScalar(const mxArray *value)
{
  return getScalar<int>(value);
}

bool Options::isLogicalScalar(const mxArray *value)
{
  return value && mxIsLogicalScalar(value);
}

bool Options::getLogicalScalar(const mxArray *value)
{
  if (isLogicalScalar(value)) return mxIsLogicalScalarTrue(value);
  if (isIntegerScalar(value)) return getIntegerScalar(value);
  if (isDoubleScalar(value))  return getDoubleScalar(value);
  return false;
}

Options::Options()
{
}

Options::Options(int nrhs, const mxArray *prhs[], bool lowerCaseKeys)
{
  init(nrhs, prhs, lowerCaseKeys);
}

Options::~Options()
{
}

void Options::init(int nrhs, const mxArray *prhs[], bool lowerCaseKeys)
{
  if (nrhs % 2 != 0) { nrhs--; prhs++; }

  for(; nrhs > 0; nrhs -= 2, prhs += 2) {
    if (!isString(prhs[0])) continue;
    std::string key = getString(prhs[0]);
    if (lowerCaseKeys) boost::algorithm::to_lower(key);

    if (isString(prhs[1])) strings_[key] = getString(prhs[1]);
    if (isDoubleScalar(prhs[1])) doubles_[key] = getDoubleScalar(prhs[1]);
    if (isLogicalScalar(prhs[1])) logicals_[key] = getLogicalScalar(prhs[1]);
  }
}

bool Options::hasKey(const std::string& key) const
{
  return strings_.count(key) || doubles_.count(key) || logicals_.count(key);
}

const std::string& Options::getString(const std::string& key, const std::string& default_value) const
{
  used_[key] = true;
  if (strings_.count(key)) return strings_.at(key);
  return default_value;
}

double Options::getDouble(const std::string& key, double default_value) const
{
  used_[key] = true;
  if (doubles_.count(key)) return doubles_.at(key);
  return default_value;
}

bool Options::getBool(const std::string& key, bool default_value) const
{
  used_[key] = true;
  if (logicals_.count(key)) return logicals_.at(key);
  if (doubles_.count(key)) return doubles_.at(key);
  return default_value;
}

Options &Options::set(const std::string& key, const std::string& value)
{
  strings_[key] = value;
  return *this;
}

Options &Options::set(const std::string& key, double value)
{
  doubles_[key] = value;
  return *this;
}

Options &Options::set(const std::string& key, bool value)
{
  logicals_[key] = value;
  return *this;
}

void Options::warnUnused() const
{
  for(std::map<std::string,std::string>::const_iterator it = strings_.begin(); it != strings_.end(); ++it)
    if (!used_[it->first]) mexPrintf(std::string("Warning: unused string argument '" + it->first + "'\n").c_str());
  for(std::map<std::string,double>::const_iterator it = doubles_.begin(); it != doubles_.end(); ++it)
    if (!used_[it->first]) mexPrintf(std::string("Warning: unused double argument '" + it->first + "'\n").c_str());
  for(std::map<std::string,bool>::const_iterator it = logicals_.begin(); it != logicals_.end(); ++it)
    if (!used_[it->first]) mexPrintf(std::string("Warning: unused logical argument '" + it->first + "'\n").c_str());
}

} // namespace rosmatlab
