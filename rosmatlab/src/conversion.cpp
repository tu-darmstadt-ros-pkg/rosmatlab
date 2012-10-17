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

#include <rosmatlab/conversion.h>
#include <introspection/message.h>
#include <introspection/type.h>

#include <ros/time.h>
#include <ros/duration.h>

#include <stdio.h>
#include <ros/message_traits.h>

#include <mex.h>

namespace rosmatlab {

Conversion::Conversion(const MessagePtr &message) : message_(message) {}
Conversion::~Conversion() {}

Array Conversion::operator()() {
  return toMatlab();
}

Array Conversion::toDoubleMatrix() {
  Array target = mxCreateDoubleMatrix(1, expanded()->size(), mxREAL);
  return toDoubleMatrix(target);
}

Array Conversion::toDoubleMatrix(Array target, std::size_t index) {
  for(Message::const_iterator field = expanded()->begin(); field != expanded()->end(); ++field) {
    if (!(*field)->getType()->isNumeric()) continue;
    mxGetPr(target)[index++] = (*field)->getType()->as_double((*field)->get());
  }
  mxSetN(target, index);
  return target;
}

Array Conversion::toStruct() {
  Array target = mxCreateStructMatrix(1, 1, message_->getFieldNames().size(), const_cast<const char **>(message_->getFieldNames().data()));
  return toStruct(target);
}

Array Conversion::toStruct(Array target, std::size_t index) {
//  mexPrintf("Constructing message %s (%s)...\n", message_->getName(), message_->getDataType());

  for(Message::const_iterator field = message_->begin(); field != message_->end(); ++field) {
    const char *field_name = (*field)->getName();

    if ((*field)->isMessage()) {
      MessagePtr field_message = (*field)->expand();

      if (field_message) {
        Array child = mxCreateStructMatrix((*field)->size(), 1, field_message->getFieldNames().size(), const_cast<const char **>(field_message->getFieldNames().data()));

        // iterate over array
        for(std::size_t j = 0; j < (*field)->size(); j++) {
//          mexPrintf("Expanding field %s[%u] (%s)... %u\n", (*field)->getName(), j, (*field)->getDataType());
          MessagePtr expanded = (*field)->expand(j);
          if (expanded) {
            Conversion(expanded).toStruct(child, j);
          } else {
            mexPrintf("Error during expansion of %s[%u] (%s)... %u\n", (*field)->getName(), j, (*field)->getDataType());
          }
        }

        mxSetField(target, index, field_name, child);

      } else {
        const char **field_names = { 0 };
        mxSetField(target, index, field_name, mxCreateStructMatrix((*field)->size(), 1, 0, field_names));
      }

    } else {
      mxSetField(target, index, field_name, convert(*field));
    }
  }

  return target;
}

Array Conversion::convert(const FieldPtr& field) {
  Array target = 0;

//  mexPrintf("Constructing field %s (%s)...\n", field->getName(), field->getDataType());
  try {
    if (field->getType() == type("string")) {
      if (field->isArray()) {
        target = mxCreateCellMatrix(field->size(), 1);
        for(std::size_t i = 0; i < field->size(); i++) {
          mxSetCell(target, i, mxCreateString(boost::any_cast<std::string>(field->get(i)).c_str()));
        }
      } else {
        target = mxCreateString(boost::any_cast<std::string>(field->get()).c_str());
      }
      return target;
    }

//    mexPrintf("Constructing double vector with dimension %u for field %s\n", unsigned(field->size()), field->getName());
    target = mxCreateDoubleMatrix(field->size(), 1, mxREAL);
    double *x = mxGetPr(target);

    if (field->getType() == type("time")) {
      for(std::size_t i = 0; i < field->size(); i++) {
        x[i] = boost::any_cast<ros::Time>(field->get(i)).toSec();
      }
      return target;
    }

    if (field->getType() == type("duration")) {
      for(std::size_t i = 0; i < field->size(); i++) {
        x[i] = boost::any_cast<ros::Time>(field->get(i)).toSec();
      }
      return target;
    }

    for(std::size_t i = 0; i < field->size(); i++) {
      x[i] = field->getType()->as<double>(field->get(i));
    }

  } catch(boost::bad_any_cast &e) {
    mexPrintf("Catched bad_any_cast exception for field %s: %s", field->getName(), e.what());
    target = emptyArray();
  }

  return target;
}

Array Conversion::emptyArray() const {
  return mxCreateDoubleMatrix(0, 0, mxREAL);
}

const MessagePtr& Conversion::expanded() {
  if (!expanded_) {
    expanded_ = expand(message_);
//    mexPrintf("Expanded an instance of %s to %u fields", message_->getDataType(), expanded_->size());
  }
  return expanded_;
}

}
