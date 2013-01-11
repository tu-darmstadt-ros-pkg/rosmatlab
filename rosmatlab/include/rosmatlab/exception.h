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

#ifndef ROSMATLAB_EXCEPTION_H
#define ROSMATLAB_EXCEPTION_H

#include <stdexcept>
#include <boost/lexical_cast.hpp>

namespace rosmatlab {

struct Exception : public std::runtime_error {
  static const std::string prefix;

  Exception(const std::runtime_error &other)
    : std::runtime_error(prefix + other.what()) {}
  Exception(const std::string &arg)
    : std::runtime_error(prefix + arg) {}
  Exception(const std::string &source, const std::string &arg)
    : std::runtime_error(prefix + source + ": " + arg) {}
};

struct ArgumentException : public Exception {
  ArgumentException(unsigned int min, unsigned int max = 0)
    : Exception("This function needs at least " + boost::lexical_cast<std::string>(min) + " argument" + (min > 1 ? "s" : "")) {}
  ArgumentException(const std::string& function, unsigned int min, unsigned int max = 0)
    : Exception(function + " needs at least " + boost::lexical_cast<std::string>(min) + " argument" + (min > 1 ? "s" : "")) {}
  ArgumentException(const std::string& function, const std::string& arg)
    : Exception(function, arg) {}
};

struct UnknownDataTypeException : public Exception {
  UnknownDataTypeException(const std::string& datatype, const std::string& text = std::string())
    : Exception("Unknown data type '" + datatype + "'" + (!text.empty() ? ". " + text : "")) {}
};

} // namespace rosmatlab

#endif // ROSMATLAB_EXCEPTION_H
