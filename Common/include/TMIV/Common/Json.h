/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _TMIV_COMMON_JSON_H_
#define _TMIV_COMMON_JSON_H_

#include <TMIV/Common/Vector.h>

#include <iosfwd>
#include <memory>
#include <string>
#include <vector>

namespace TMIV::Common {
namespace impl {
struct Value;
struct Object;
} // namespace impl

class Json {
public:
  enum class Type { number, string, array, object, boolean, null };

  // Initialize as a null node
  Json();

  // Initialize from an input stream
  explicit Json(std::istream &stream);

  // For a Json of type Object specify another Json of Type Object that
  // overrides this one for all keys
  void setOverrides(const Json &overrides);

  Type type() const;
  Json optional(std::string const &key) const;
  Json require(std::string const &key) const;
  bool isPresent(std::string const &key) const;

  // Index into an array
  Json at(size_t index) const;

  // Return the number of elements in an object or array
  size_t size() const;

  double asDouble() const;
  float asFloat() const;
  int asInt() const;
  std::string const &asString() const;
  bool asBool() const;
  auto asStringVector() const -> std::vector<std::string>;
  template <stack::size_type M> auto asIntVector() const -> stack::Vector<int, M>;
  template <stack::size_type M> auto asFloatVector() const -> stack::Vector<float, M>;

  // Anything apart from false and null is true
  explicit operator bool() const;

private:
  explicit Json(std::shared_ptr<impl::Value> value);

  std::shared_ptr<impl::Value> m_value;
};
} // namespace TMIV::Common

#include "Json.hpp"

#endif
