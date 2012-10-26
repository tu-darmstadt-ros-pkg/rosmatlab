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

#ifndef ROSMATLAB_CONVERSION_H
#define ROSMATLAB_CONVERSION_H

#include <introspection/forwards.h>
#include "matrix.h"

namespace rosmatlab {

using namespace cpp_introspection;

class Conversion;
typedef boost::shared_ptr<Conversion> ConversionPtr;

typedef mxArray *Array;
typedef mxArray const *ConstArray;

class Conversion {
public:
  Conversion(const MessagePtr &message);
  virtual ~Conversion();

  virtual Array toMatlab() { return toStruct(); }

  virtual Array toDoubleMatrix();
  virtual Array toDoubleMatrix(Array target, std::size_t n = 0);

  virtual Array toStruct();
  virtual Array toStruct(Array target, std::size_t index = 0);

  virtual MessagePtr fromMatlab(ConstArray source, std::size_t index = 0);
  virtual void fromMatlab(const MessagePtr &message, ConstArray source, std::size_t index = 0);

  virtual Array convertToMatlab(const FieldPtr& field);
  virtual void convertFromMatlab(const FieldPtr& field, ConstArray source);
  virtual const double *convertFromDouble(const FieldPtr& field, const double *begin, const double *end);

  const MessagePtr& expanded();

protected:
  Array emptyArray() const;

  virtual void fromDoubleMatrix(const MessagePtr &message, ConstArray source, std::size_t n = 0);
  virtual void fromDoubleMatrix(const MessagePtr &message, const double *begin, const double *end);
  virtual void fromStruct(const MessagePtr &message, ConstArray source, std::size_t index = 0);

  MessagePtr message_;
  MessagePtr expanded_;
};

}

#endif // ROSMATLAB_CONVERSION_H
