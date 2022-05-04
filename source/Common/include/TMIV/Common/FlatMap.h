/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#ifndef TMIV_COMMON_FLAT_MAP_H
#define TMIV_COMMON_FLAT_MAP_H

#include "verify.h"

#include <vector>

namespace TMIV::Common {
template <typename Key, typename Value> struct KeyValuePair {
  explicit KeyValuePair(Key key_) : key{std::move(key_)} {}
  KeyValuePair(Key key_, Value value_) : key{std::move(key_)}, value{std::move(value_)} {}

  Key key;
  Value value{};

  [[nodiscard]] constexpr auto operator==(const KeyValuePair &other) const {
    return key == other.key && value == other.value;
  }

  [[nodiscard]] constexpr auto operator!=(const KeyValuePair &other) const {
    return !operator==(other);
  }
};

template <typename Key, typename Value>
class FlatMap : public std::vector<KeyValuePair<Key, Value>> {
public:
  using std::vector<KeyValuePair<Key, Value>>::operator[];

  // Return the value of the first occurrence of the specified key.
  [[nodiscard]] auto operator[](const Key &target) const noexcept -> const auto & {
    for (auto &kvp : *this) {
      if (target == kvp.key) {
        return kvp.value;
      }
    }
    UNREACHABLE;
  }

  // Find the first occurrence of the specified key. When missing, a key-value pair is added.
  auto operator[](const Key &target) -> auto & {
    for (auto &kvp : *this) {
      if (target == kvp.key) {
        return kvp.value;
      }
    }
    return this->emplace_back(target, Value{}).value;
  }

  [[nodiscard]] auto contains(const Key &target) const noexcept {
    return std::any_of(this->cbegin(), this->cend(),
                       [&target](const auto &kvp) { return kvp.key == target; });
  }
};
} // namespace TMIV::Common

#endif
