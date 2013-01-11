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

#ifndef ROSMATLAB_OPTIONS_H
#define ROSMATLAB_OPTIONS_H

#include <matrix.h>
#include <string>
#include <map>
#include <vector>

namespace rosmatlab {
class Options {
public:

  static bool isString(const mxArray *value);
  static std::string getString(const mxArray *value);

  static bool isScalar(const mxArray *value);

  static bool isDoubleScalar(const mxArray *value);
  static double getDoubleScalar(const mxArray *value);

  static bool isIntegerScalar(const mxArray *value);
  static int getIntegerScalar(const mxArray *value);

  static bool isLogicalScalar(const mxArray *value);
  static bool getLogicalScalar(const mxArray *value);

public:
  typedef std::vector<std::string> Strings;
  typedef std::map<std::string, Strings> StringMap;
  typedef std::vector<double> Doubles;
  typedef std::map<std::string, Doubles> DoubleMap;
  typedef std::vector<bool> Bools;
  typedef std::map<std::string, Bools> BoolMap;

public:
  Options();
  Options(int nrhs, const mxArray *prhs[], bool lowerCaseKeys = false);
  virtual ~Options();

  void init(int nrhs, const mxArray *prhs[], bool lowerCaseKeys = false);

  bool hasKey(const std::string& key) const;

  const std::string& getString(const std::string& key, const std::string& default_value = std::string()) const;
  const Strings& getStrings(const std::string& key) const;
  double getDouble(const std::string& key, double default_value = 0) const;
  const Doubles& getDoubles(const std::string& key) const;
  bool getBool(const std::string& key, bool default_value = false) const;
  const Bools& getBools(const std::string& key) const;

  Options &set(const std::string &key, const mxArray *value);
  Options &set(const std::string& key, const std::string& value);
  Options &set(const std::string& key, double value);
  Options &set(const std::string& key, bool value);

  void warnUnused() const;

  const StringMap& strings() const { return strings_; }
  const DoubleMap& doubles() const { return doubles_; }
  const BoolMap& bools() const { return bools_; }

private:
  StringMap strings_;
  DoubleMap doubles_;
  BoolMap bools_;

  mutable std::map<std::string,bool> used_;
};

} // namespace rosmatlab

#endif // ROSMATLAB_OPTIONS_H
