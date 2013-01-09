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

#ifndef ROSMATLAB_LOG_H
#define ROSMATLAB_LOG_H

#include <matrix.h>
#include <ros/console.h>

namespace rosmatlab {
namespace log {

using namespace ::ros::console::levels;

void log(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void log(Level level, int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void log(Level level, const char *name, const char *fmt, ...);

} // namespace log
} // namespace rosmatlab

#define ROSMATLAB_DEBUG(...) ::rosmatlab::log::log(::rosmatlab::log::Debug, 0, __VA_ARGS__)
#define ROSMATLAB_INFO(...)  ::rosmatlab::log::log(::rosmatlab::log::Info,  0, __VA_ARGS__)
#define ROSMATLAB_WARN(...)  ::rosmatlab::log::log(::rosmatlab::log::Warn,  0, __VA_ARGS__)
#define ROSMATLAB_ERROR(...) ::rosmatlab::log::log(::rosmatlab::log::Error, 0, __VA_ARGS__)
#define ROSMATLAB_FATAL(...) ::rosmatlab::log::log(::rosmatlab::log::Fatal, 0, __VA_ARGS__)

#define ROSMATLAB_DEBUG_NAMED(name, ...) ::rosmatlab::log::log(::rosmatlab::log::Debug, name, __VA_ARGS__)
#define ROSMATLAB_INFO_NAMED(name, ...)  ::rosmatlab::log::log(::rosmatlab::log::Info,  name, __VA_ARGS__)
#define ROSMATLAB_WARN_NAMED(name, ...)  ::rosmatlab::log::log(::rosmatlab::log::Warn,  name, __VA_ARGS__)
#define ROSMATLAB_ERROR_NAMED(name, ...) ::rosmatlab::log::log(::rosmatlab::log::Error, name, __VA_ARGS__)
#define ROSMATLAB_FATAL_NAMED(name, ...) ::rosmatlab::log::log(::rosmatlab::log::Fatal, name, __VA_ARGS__)

#endif // ROSMATLAB_LOG_H
