/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#ifndef TMIV_COMMON_FORMATTERS_H
#define TMIV_COMMON_FORMATTERS_H

#include <TMIV/Common/format.h>

#include "Json.h"
#include "LoggingStrategy.h"
#include "Vector.h"

#include <filesystem>
#include <sstream>

namespace TMIV::Common {
// https://fmt.dev/latest/api.html?highlight=ostream_formatter
class OstreamFormatter {
public:
  template <class Context> constexpr auto parse(Context &ctx) { return m_formatter.parse(ctx); }

  template <class Value, class Context> auto format(Value &&value, Context &ctx) const {
    std::ostringstream stream;
    stream << std::forward<Value>(value);
    return m_formatter.format(std::move(stream).str(), ctx);
  }

private:
  TMIV_FMT::formatter<std::string> m_formatter;
};
} // namespace TMIV::Common

namespace TMIV_FMT {
template <> struct formatter<TMIV::Common::LogLevel> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<std::filesystem::path> : TMIV::Common::OstreamFormatter {};
template <typename T, size_t N>
struct formatter<TMIV::Common::stack::Vector<T, N>> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<std::streampos> : TMIV::Common::OstreamFormatter {};
} // namespace TMIV_FMT

namespace TMIV::Common {
template <typename Idc, size_t N>
auto queryEnum(const Json &node, const std::string &key, const std::string &name,
               const std::array<Idc, N> &known) {
  const auto text = node.require(key).as<std::string>();

  for (auto i : known) {
    if (TMIV_FMT::format("{}", i) == text) {
      return i;
    }
  }
  throw std::runtime_error(TMIV_FMT::format("The configured {} IDC {} is unknown", name, text));
}
} // namespace TMIV::Common

#endif
