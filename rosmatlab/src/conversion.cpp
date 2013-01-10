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
#include <rosmatlab/exception.h>

#include <introspection/message.h>
#include <introspection/type.h>

#include <ros/time.h>
#include <ros/duration.h>

#include <stdio.h>
#include <ros/message_traits.h>

#include <mex.h>

namespace rosmatlab {

Conversion::Conversion(const MessagePtr &message) : message_(message), options_(defaultOptions()) {}
Conversion::Conversion(const MessagePtr &message, const Options& options) : message_(message), options_(options) {}
Conversion::~Conversion() {}

Array Conversion::toMatlab() {
  return toStruct();
}

Array Conversion::toDoubleMatrix() {
  return toDoubleMatrix(0);
}

Array Conversion::toDoubleMatrix(Array target, std::size_t n) {
  std::size_t m = 0;
  if (!target) target = mxCreateDoubleMatrix(expanded()->size(), n + 1, mxREAL);

  bool do_realloc = false;
  if (mxGetM(target) < expanded()->size()) { mxSetM(target, expanded()->size()); do_realloc = true; }
  if (mxGetN(target) < n + 1) { mxSetN(target, n + 1); do_realloc = true; }
  if (do_realloc)
  {
    mxSetData(target, mxRealloc(mxGetData(target), mxGetN(target) * mxGetN(target) * sizeof(double)));
  }

  double *data = mxGetPr(target) + mxGetM(target) * n;
  for(Message::const_iterator field = expanded()->begin(); field != expanded()->end(); ++field) {
    if (!(*field)->getType()->isNumeric()) continue;
    *data++ = (*field)->getType()->as_double((*field)->get());
  }
  return target;
}

Array Conversion::toStruct() {
  return toStruct(0);
}

Array Conversion::toStruct(Array target, std::size_t index) {
//  mexPrintf("Constructing message %s (%s)...\n", message_->getName(), message_->getDataType());
  if (!target) target = mxCreateStructMatrix(1, index + 1, message_->getFieldNames().size(), const_cast<const char **>(message_->getFieldNames().data()));

  for(Message::const_iterator field = message_->begin(); field != message_->end(); ++field) {
    const char *field_name = (*field)->getName();

    if ((*field)->isMessage()) {
      MessagePtr field_message = messageByDataType((*field)->getDataType());

      if (field_message) {
        Array child = mxCreateStructMatrix(1, (*field)->size(), field_message->getFieldNames().size(), const_cast<const char **>(field_message->getFieldNames().data()));

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
        mxSetField(target, index, field_name, mxCreateStructMatrix(1, (*field)->size(), 0, field_names));
      }

    } else {
      mxSetField(target, index, field_name, convertToMatlab(*field));
    }
  }

  // add meta data to the struct
  {
    if (mxGetFieldNumber(target, "DATATYPE") == -1) mxAddField(target, "DATATYPE");
    mxSetField(target, index, "DATATYPE", mxCreateString(message_->getDataType()));
    if (mxGetFieldNumber(target, "MD5SUM") == -1) mxAddField(target, "MD5SUM");
    mxSetField(target, index, "MD5SUM", mxCreateString(message_->getMD5Sum()));
  }

  return target;
}

std::size_t Conversion::numberOfInstances(ConstArray source)
{
  if (mxIsStruct(source)) {
    return mxGetNumberOfElements(source);
  }

  if (mxIsDouble(source)) {
    return mxGetM(source) == 1 ? mxGetN(source) : mxGetM(source);
  }

  if (mxIsChar(source)) {
    return 1;
  }

  throw Exception("Cannot parse an array of class " + std::string(mxGetClassName(source)) + " as ROS message");
}

MessagePtr Conversion::fromMatlab(ConstArray source, std::size_t index)
{
  MessagePtr target = message_->introspect(message_->createInstance());
  fromMatlab(target, source, index);
  return target;
}

void Conversion::fromMatlab(const MessagePtr& target, ConstArray source, std::size_t index)
{
  if (mxIsStruct(source)) {
    fromStruct(target, source, index);
    return;
  }

  if (mxIsDouble(source)) {
    fromDoubleMatrix(target, source, index);
    return;
  }

  if (mxIsChar(source) && target->hasType<std_msgs::String>()) {
    std_msgs::StringPtr data = target->getInstanceAs<std_msgs::String>();
    if (data) {
      data->data = Options::getString(source);
      return;
    }
  }

  throw Exception("Cannot parse an array of class " + std::string(mxGetClassName(source)) + " as " + std::string(target->getDataType()) + " message");
}

void Conversion::fromDoubleMatrix(const MessagePtr& target, ConstArray source, std::size_t n)
{
  if (!mxIsDouble(source)) return;

  const double *begin = 0;
  const double *end = 0;

  if (mxGetM(source) == 1 && n == 0) {
    begin = mxGetPr(source);
    end   = begin + mxGetN(source);
  } else {
    if (n >= mxGetN(source)) throw Exception("Column index out of bounds");
    begin = mxGetPr(source) + mxGetM(source) * n;
    end   = mxGetPr(source) + mxGetM(source) * (n + 1);
  }

  fromDoubleMatrix(target, begin, end);
}

void Conversion::fromDoubleMatrix(const MessagePtr &target, const double *begin, const double *end)
{
  for(Message::const_iterator field = target->begin(); field != target->end(); ++field) {
    begin = convertFromDouble(*field, begin, end);
  }
  if (begin != end) throw Exception("Failed to parse an array of type " + std::string(target->getDataType()) + ": vector is too long");
}

void Conversion::fromStruct(const MessagePtr &target, ConstArray source, std::size_t index)
{
  if (!mxIsStruct(source)) return;
  if (index >= mxGetNumberOfElements(source)) throw Exception("Index out of bounds");

  for(Message::const_iterator field = target->begin(); field != target->end(); ++field) {
    ConstArray field_source = mxGetField(source, index, (*field)->getName());
    if (!field_source) continue;
    convertFromMatlab(*field, field_source);
  }
}

Array Conversion::convertToMatlab(const FieldPtr& field) {
  Array target = 0;
  TypePtr field_type = field->getType();

//  mexPrintf("Constructing field %s (%s)...\n", field->getName(), field->getDataType());
  try {
    if (field_type->isString()) {
      if (field->isArray()) {
        target = mxCreateCellMatrix(1, field->size());
        for(std::size_t i = 0; i < field->size(); i++) {
          mxSetCell(target, i, mxCreateString(field_type->as_string(field->get(i)).c_str()));
        }
      } else {
        target = mxCreateString(field_type->as_string(field->get()).c_str());
      }
      return target;
    }

//    mexPrintf("Constructing double vector with dimension %u for field %s\n", unsigned(field->size()), field->getName());
    target = mxCreateDoubleMatrix(1, field->size(), mxREAL);
    double *x = mxGetPr(target);

    for(std::size_t i = 0; i < field->size(); i++) {
      x[i] = field_type->as_double(field->get(i));
    }

  } catch(boost::bad_any_cast &e) {
    mexPrintf("Catched bad_any_cast exception for field %s: %s", field->getName(), e.what());
    target = emptyArray();
  }

  return target;
}

void Conversion::convertFromMatlab(const FieldPtr &field, ConstArray source) {
  const char *field_name = field->getName();
  std::size_t field_size = mxIsChar(source) ? 1 : mxGetNumberOfElements(source);

  // check size
  if (field->isArray() && field->size() != field_size) throw Exception("Failed to parse field " + std::string(field_name) + ": Array field must have length " + boost::lexical_cast<std::string>(field->size()));
  if (!field->isContainer() && field_size != 1) throw Exception("Failed to parse field " + std::string(field_name) + ": Scalar field must have exactly length 1");
  if (field->isVector()) field->resize(field_size);

  // parse ROS message
  if (field->isMessage()) {
    MessagePtr field_message = messageByDataType(field->getDataType());
    if (!field_message) throw Exception("Failed to parse field " + std::string(field_name) + ": unknown datatype " + field->getDataType());
    Conversion child_conversion(field_message);

    // iterate over array
    for(std::size_t i = 0; i < field_size; i++) {
//     mexPrintf("Expanding field %s[%u] (%s)... %u\n", field->getName(), i, field->getDataType());
      MessagePtr expanded = field->expand(i);
      if (expanded) {
        child_conversion.fromMatlab(expanded, source, i);
      } else {
        mexPrintf("Error during expansion of %s[%u] (%s)... %u\n", field->getName(), i, field->getDataType());
      }
    }

    return;
  }

  // parse string
  if (field->getType()->isString()) {
    std::vector<char> buffer;

    for(std::size_t i = 0; i < field->size(); i++) {
      if (mxIsCell(source) && mxIsChar(mxGetCell(source, i))) {
        buffer.resize(mxGetNumberOfElements(mxGetCell(source, i)) + 1);
        mxGetString(mxGetCell(source, i), buffer.data(), buffer.size());
        field->set(std::string(buffer.data(), buffer.size() - 1), i);

      } else if (mxIsChar(source) && i == 0) {
        buffer.resize(mxGetN(source) + 1);
        mxGetString(source, buffer.data(), buffer.size());
        field->set(std::string(buffer.data(), buffer.size() - 1));

      } else {
        throw Exception("Failed to parse string field " + std::string(field->getDataType()) + " " + std::string(field->getName()) + ": Array must be a cell string or a character array");
      }
    }

    return;
  }

  // For all other types source must be a double array...
  if (!mxIsDouble(source)) throw Exception("Failed to parse field " + std::string(field->getDataType()) + " " + std::string(field->getName()) + ": Array must be a double array");
  const double *x = mxGetPr(source);
  convertFromDouble(field, x, x + mxGetN(source));
}

const double *Conversion::convertFromDouble(const FieldPtr& field, const double *begin, const double *end)
{
  const double *data = begin;

  if (field->isVector() && end >= begin) {
    // eat up all the input (roar!!!)
    field->resize(end - begin);
  }

  // check size
  if (end - begin < field->size()) {
    throw Exception("Failed to parse field " + std::string(field->getName()) + ": vector is too short");
  }

  // parse ROS message
  if (field->isMessage()) {
    // TODO
  }

  // ignore strings (set to empty string)
  if (field->getType()->isString()) {
    for(std::size_t i = 0; i < field->size(); i++) {
      field->set(std::string(), i);
      data++;
    }
    return data;
  }

  // read doubles
  for(std::size_t i = 0; i < field->size(); i++) {
    field->set(*data++, i);
  }

  return data;
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

Options &Conversion::defaultOptions() {
  static Options default_options_;
  return default_options_;
}

}
