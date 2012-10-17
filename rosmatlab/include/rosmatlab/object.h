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

#ifndef ROSMATLAB_OBJECT_H
#define ROSMATLAB_OBJECT_H

#include <rosmatlab/exception.h>
#include <boost/shared_ptr.hpp>
#include "mex.h"

#include <stdint.h>

namespace rosmatlab {

namespace {
  struct null_deleter {
    void operator()(void const *) const {}
  };
}

template <class Type>
class Object
{
public:
  typedef boost::shared_ptr<Type> Ptr;

  Object() { construct(); }
  Object(Type *instance) { *this = instance; construct(); }
  Object(const Ptr &instance) { *this = instance; construct(); }
  Object(const Object &other) { *this = other; construct(); }
  virtual ~Object() { if (handle_) mxDestroyArray(handle_); }

  const Ptr &instance() const { return instance_; }
  mxArray *handle() const { return handle_; }

  Type* get() { return instance_.get(); }
  const Type* get() const { return instance_.get(); }
  Type &operator*() { return *instance_; }
  const Type &operator*() const { return *instance_; }

  Object<Type> &operator=(const Object &other) {
    return *this = other.instance_;
  }

  Object<Type> &operator=(Type *instance) {
    return *this = Ptr(instance, null_deleter());
  }

  Object<Type> &operator=(const Ptr &instance) {
    instance_ = instance;
    return *this;
  }

  static Object<Type> *byHandle(const mxArray *handle) {
    const mxArray *ptr = 0;
    if (!handle) return 0;
    // mexPrintf("Searching for object of type %s (class %s)...\n", typeid(Type).name(), mxGetClassName(handle));
    if (mxIsClass(handle, class_name_)) {
      ptr = mxGetProperty(handle, 0, "handle");
    } else if (mxIsStruct(handle)) {
      ptr = mxGetField(handle, 0, "handle");
    } else if (mxIsDouble(handle)) {
      ptr = handle;
    }
    if (!ptr || !mxIsDouble(ptr) || !mxGetNumberOfElements(ptr) > 0 || !mxGetPr(ptr)) throw Exception("invalid handle");

    Object<Type> *object = reinterpret_cast<Object<Type> *>(static_cast<uint64_t>(*mxGetPr(ptr)));
    return object;
  }

private:
  boost::shared_ptr<Type> instance_;
  mxArray *handle_;
  static const char *class_name_;

  void construct() {
    handle_ = mxCreateDoubleScalar(reinterpret_cast<uint64_t>(this));
    mexMakeArrayPersistent(handle_);
    assert(byHandle(handle()) == this);
  }
};

template <class Type>
Type *getObject(const mxArray *handle) {
  Object<Type> *object = Object<Type>::byHandle(handle);
  if (!object) return 0;
  return object->get();
}

} // namespace rosmatlab

#endif // ROSMATLAB_OBJECT_H
