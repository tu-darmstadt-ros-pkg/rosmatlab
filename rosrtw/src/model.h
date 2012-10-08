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

#ifndef ROSRTW_MODEL_H
#define ROSRTW_MODEL_H

#include "grt.h"
#include <rosrtw/model_interface.h>

namespace rosrtw {

class Model : public ModelInterface {
private:
  RT_MODEL  *S;
  real_T     finaltime;

  const std::string name;
  bool is_initialized;

  struct {
    int_T    stopExecutionFlag;
    int_T    isrOverrun;
    int_T    overrunFlags[NUMST];
    int_T    eventFlags[NUMST];
    const    char_T *errmsg;
  } GBLbuf;

private:
  void rt_OneStep(RT_MODEL *S);

public:
  Model();
  virtual ~Model();

  const std::string& getName() const { return name; }
  bool isInitialized() const { return is_initialized; }

  bool isRunning();
  bool initialize();
  void loop();
  void step();
  void stop();
  void terminate();
};

} // namespace rosrtw

#endif // ROSRTW_MODEL_H
