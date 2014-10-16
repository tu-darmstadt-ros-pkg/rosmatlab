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

#include <rosmatlab/message.h>
#include <rosmatlab/conversion.h>
#include <rosmatlab/options.h>
#include <rosmatlab/log.h>
#include <rosmatlab/exception.h>

#include <introspection/message.h>

#include <mex.h>

#include <boost/algorithm/string.hpp>

namespace rosmatlab {

mxArray *message_constructor(const MessagePtr& message, int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  mxArray *result = 0;

  // parse inputs
  if (nrhs > 0 && Options::isString(prhs[0])) {
    std::string option = Options::getString(prhs[0]);
    if (nrhs == 1 && boost::algorithm::iequals(option, "datatype")) {
      result = mxCreateString(message->getDataType());
      return result;
    }
    if (nrhs == 1 && boost::algorithm::iequals(option, "definition")) {
      result = mxCreateString(message->getDefinition());
      return result;
    }
    if (nrhs == 1 && boost::algorithm::iequals(option, "md5sum")) {
      result = mxCreateString(message->getMD5Sum());
      return result;
    }
    if (nrhs == 1 && boost::algorithm::iequals(option, "fields")) {
      result = mxCreateCellMatrix(1, message->getFieldNames().size());
      for(std::size_t i = 0; i < message->getFieldNames().size(); ++i) {
        mxSetCell(result, i, mxCreateString(message->getFieldNames().at(i)));
      }
      return result;
    }
    if ((nrhs == 1 || nrhs == 2) && boost::algorithm::iequals(option, "default")) {
      if (nrhs == 2) {
        const mxArray *default_options = prhs[1];
        Conversion::perMessageOptions(message).merge(ConversionOptions(1, &default_options));
      }
      return Conversion::perMessageOptions(message).toMatlab();
    }
  }

  // Check if a numeric argument (message count) has been given
  std::size_t count = 1;
  if (nrhs > 0 && (Options::isIntegerScalar(prhs[0]) || Options::isDoubleScalar(prhs[0]))) {
    count = Options::getIntegerScalar(prhs[0]);
    nrhs--; prhs++;
  }

  // Get conversion options
  ConversionOptions options;
  if (nrhs % 2 == 0) {
    options.init(nrhs, prhs);
  } else {
    options.init(nrhs - 1, prhs + 1);
  }

  // Is this a copy constructor?
  if (nrhs % 2 != 0) {
    Conversion conversion(message, options);
    V_Message copy(conversion.numberOfInstances(prhs[0]));

    for(std::size_t j = 0; j < copy.size(); j++) {
      copy[j] = conversion.fromMatlab(prhs[0], j);
      // std::cout << "Constructed a new " << copy[j]->getDataType() << " message: " << *boost::shared_static_cast<MessageType const>(copy[j]->getConstInstance()) << std::endl;
      result = Conversion(conversion, copy[j]).toMatlab(result, j, copy.size());
    }

  // otherwise construct a new message
  } else {
    MessagePtr m = message->introspect(message->createInstance());
    // std::cout << "Constructed a new " << m->getDataType() << " message: " << *boost::shared_static_cast<MessageType const>(m->getConstInstance()) << std::endl;
    Conversion conversion(m, options);
    result = conversion.toMatlab();
  }

  // copy the contents of result if count > 1
  if (count > 1) {
    // This seems to be a bit hacky. Is there a better solution?
    mxArray *repmatrhs[] = { result, mxCreateDoubleScalar(1), mxCreateDoubleScalar(count) };
    mxArray *repmatlhs[] = { 0 };
    mexCallMATLAB(1, repmatlhs, 3, repmatrhs, "repmat");
    mxDestroyArray(repmatrhs[0]);
    mxDestroyArray(repmatrhs[1]);
    mxDestroyArray(repmatrhs[2]);
    result = repmatlhs[0];
  }

  return result;
}

} // namespace rosmatlab
