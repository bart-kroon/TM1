/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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

#ifndef _TMIV_COMMON_COMMON_H_
#error "Include the .h instead of the .hpp."
#endif

#include <cassert>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>

namespace TMIV::Common {
namespace detail {
inline auto findReplacementField(const std::string &fmt, size_t pos)
    -> std::optional<std::pair<std::size_t, std::size_t>> {
  const auto open = fmt.find('{', pos);
  if (open == std::string::npos) {
    return {};
  }

  const auto close = fmt.find('}', open + 1);
  if (close == std::string::npos) {
    std::ostringstream what;
    what << "Missing '}' in format(fmt, ...) with fmt=\"" << fmt << '"';
    throw std::runtime_error(what.str());
  }

  return std::pair{open, close + 1};
}

// Extend as needed. So far the format function is only used for filename patterns with names, ID's
// and frame sizes. A more complete implementation of formatReplacementField<Arg> could use
// std::regex to parse the field.
template <typename Arg>
void formatReplacementField(std::ostream &stream, const std::string &field, Arg &&arg) {
  using namespace std::string_literals;
  const auto fill = stream.fill('0');
  if (field == "{}"s) {
    stream << std::forward<Arg>(arg);
  } else if (field == "{:02}"s) {
    stream << std::setw(2) << std::forward<Arg>(arg);
  } else if (field == "{:03}"s) { // enough for fixed width printing of uint8_t
    stream << std::setw(3) << std::forward<Arg>(arg);
  } else if (field == "{:04}"s) {
    stream << std::setw(4) << std::forward<Arg>(arg);
  } else if (field == "{:05}"s) { // enough for fixed width printing of uint16_t
    stream << std::setw(5) << std::forward<Arg>(arg);
  } else {
    std::ostringstream what;
    what << "Incorrect or unsupported replacement field " << field;
    throw std::runtime_error(what.str());
  }
  stream.fill(fill);
}

inline void format(std::ostream &stream, const std::string &fmt, size_t pos) {
  if (findReplacementField(fmt, pos)) {
    std::ostringstream what;
    what << "Not enough arguments provided to format(fmt, ...) with fmt=\"" << fmt << '"';
    throw std::runtime_error(what.str());
  }

  for (; pos < fmt.size(); ++pos) {
    stream.put(fmt[pos]);
  }
}

template <typename Arg0, typename... Args>
void format(std::ostream &stream, const std::string &fmt, size_t pos, Arg0 &&arg0,
            Args &&... args) {
  const auto f = findReplacementField(fmt, pos);

  if (!f) {
    stream << fmt.substr(pos);
    return;
  }

  assert(pos <= f->first && f->first < f->second && f->second <= fmt.size());

  for (; pos < f->first; ++pos) {
    stream.put(fmt[pos]);
  }

  formatReplacementField(stream, fmt.substr(f->first, f->second - f->first),
                         std::forward<Arg0>(arg0));
  format(stream, fmt, f->second, std::forward<Args>(args)...);
}
} // namespace detail

template <typename... Args> auto format(const std::string &fmt, Args &&... args) -> std::string {
  std::ostringstream stream;
  detail::format(stream, fmt, 0, std::forward<Args>(args)...);
  return stream.str();
}

inline constexpr unsigned maxLevel(unsigned bits) { return (1U << bits) - 1U; }

template <unsigned bits> float expandValue(uint16_t x) { return float(x) / float(maxLevel(bits)); }

template <unsigned bits> uint16_t quantizeValue(float x) {
  if (x >= 0.F && x <= 1.F) {
    return static_cast<uint16_t>(
        std::min(unsigned(std::lround(x * float(maxLevel(bits)))), maxLevel(bits)));
  }
  if (x > 0) {
    return static_cast<uint16_t>(maxLevel(bits));
  }
  return 0;
}
} // namespace TMIV::Common
