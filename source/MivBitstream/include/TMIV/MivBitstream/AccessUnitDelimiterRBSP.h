/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#ifndef TMIV_MIVBITSTREAM_ACCESSUNITDELIMITERRBSP_H
#define TMIV_MIVBITSTREAM_ACCESSUNITDELIMITERRBSP_H

#include <TMIV/Common/Bitstream.h>

namespace TMIV::MivBitstream {
// 23090-5: aframe_type
enum class AframeType : uint8_t { I, P_and_I, SKIP_P_and_I, SKIP };
auto operator<<(std::ostream & /*stream*/, AframeType /*x*/) -> std::ostream &;

// 23090-5: access_unit_delimiter_rbsp()
class AccessUnitDelimiterRBSP {
public:
  AccessUnitDelimiterRBSP() = default;
  explicit constexpr AccessUnitDelimiterRBSP(AframeType aframe_type);

  [[nodiscard]] constexpr auto aframe_type() const noexcept;
  constexpr auto aframe_type(AframeType value) noexcept;

  friend auto operator<<(std::ostream &stream, const AccessUnitDelimiterRBSP &x) -> std::ostream &;

  constexpr auto operator==(const AccessUnitDelimiterRBSP &other) const noexcept;
  constexpr auto operator!=(const AccessUnitDelimiterRBSP &other) const noexcept;

  static auto decodeFrom(std::istream &stream) -> AccessUnitDelimiterRBSP;

  void encodeTo(std::ostream &stream) const;

private:
  AframeType m_aframe_type{};
};
} // namespace TMIV::MivBitstream

#include "AccessUnitDelimiterRBSP.hpp"

#endif
