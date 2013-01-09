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

#include <mex.h>
#include <rosmatlab/log.h>
#include <rosmatlab/exception.h>
#include <rosmatlab/options.h>

#include <ros/console.h>
#include <boost/algorithm/string.hpp>
#include <cstdarg>
#include <boost/shared_array.hpp>

namespace rosmatlab {
namespace log {

#define INITIAL_BUFFER_SIZE 4096
static boost::shared_array<char> g_print_buffer(new char[INITIAL_BUFFER_SIZE]);
static size_t g_print_buffer_size = INITIAL_BUFFER_SIZE;

static bool getLevel(const std::string& str, Level &level) {
  if (boost::iequals(str, "debug")) { level = Debug; return true; }
  if (boost::iequals(str, "info"))  { level = Info;  return true; }
  if (boost::iequals(str, "warn"))  { level = Warn;  return true; }
  if (boost::iequals(str, "error")) { level = Error; return true; }
  if (boost::iequals(str, "fatal")) { level = Fatal; return true; }
  return false;
}

static bool getLevel(double dbl, Level &level) {
  unsigned int i = static_cast<unsigned int>(dbl);
  if (i >= Count) return false;
  level = static_cast<Level>(i);
  return true;
}

void log(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  Level level = ::ros::console::levels::Info;
  std::string name;
  std::string message;

  // parse input arguments
  if (nrhs == 1) {
    message = Options::getString(prhs[0]);
  } else if (nrhs == 2) {
    std::string arg1 = Options::getString(prhs[0]);
    if (!getLevel(arg1, level) && !getLevel(Options::getDoubleScalar(prhs[0]), level)) name = arg1;
    message = Options::getString(prhs[1]);
  } else if (nrhs > 2) {
    if (!getLevel(Options::getString(prhs[0]), level) && !getLevel(Options::getDoubleScalar(prhs[0]), level)) {
      throw Exception("Could not evaluate first argument as log level (Debug, Info, Warn, Error, Fatal)");
    }
    name = Options::getString(prhs[1]);
    message = Options::getString(prhs[2]);
  }

  log(level, name.c_str(), message.c_str());
}

void log(Level level, int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  std::string name;
  std::string message;

  // parse input arguments
  if (nrhs == 1) {
    message = Options::getString(prhs[0]);
  } else if (nrhs == 2) {
    name = Options::getString(prhs[0]);
    message = Options::getString(prhs[1]);
  }

  log(level, name.c_str(), message.c_str());
}

void log(Level level, const char *name, const char *fmt, ...)
{
  std::string file;
  std::string function;
  int line = 0;

  va_list args;
  va_start(args, fmt);
  ros::console::vformatToBuffer(g_print_buffer, g_print_buffer_size, fmt, args);
  va_end(args);
  std::stringstream message(std::string(g_print_buffer.get(), g_print_buffer_size));

  // get stack information from matlab
  mxArray *stack;
  mexCallMATLAB(1, &stack, 0, 0, "dbstack");
  if (mxIsStruct(stack) && mxGetNumberOfElements(stack) > 0) {
    file     = Options::getString(mxGetField(stack, 0, "file"));
    function = Options::getString(mxGetField(stack, 0, "name"));
    line     = static_cast<int>(Options::getDoubleScalar(mxGetField(stack, 0, "line")));
  }

  // expand logger name
  std::string logger_name;
  if (name && strlen(name) > 0) {
    logger_name = std::string(ROSCONSOLE_DEFAULT_NAME) + "." + name;
  } else {
    logger_name = std::string(ROSCONSOLE_DEFAULT_NAME);
  }

  // ... and finally log the message!
  do {
    ROSCONSOLE_DEFINE_LOCATION(true, level, logger_name);
    if (ROS_UNLIKELY(enabled))
    {
      ::ros::console::print(0, loc.logger_, loc.level_, message, file.c_str(), line, function.c_str());
    }

    if (enabled) {
      static const std::string level_strings[] = { "Debug", "Info", "Warn", "Error", "Fatal" };
      std::stringstream output;
      output << "[" << level_strings[level] << "] [" << ros::WallTime::now() << "] " << message.str();
      mexPrintf(output.str().c_str());
      mexPrintf("\n");
    }
  } while(0);
}

} // namespace log
} // namespace rosmatlab
