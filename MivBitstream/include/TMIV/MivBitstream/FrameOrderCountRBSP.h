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

#ifndef _TMIV_MIVBITSTREAM_FRAMEORDERCOUNTRBSP_H_
#define _TMIV_MIVBITSTREAM_FRAMEORDERCOUNTRBSP_H_

#include <TMIV/MivBitstream/AtlasAdaptationParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>

#include <TMIV/Common/Bitstream.h>

#include <cstdint>
#include <cstdlib>
#include <iosfwd>

namespace TMIV::MivBitstream {
// 23090-5: frame_order_count_rbsp( )
class FrameOrderCountRBSP {
public:
  FrameOrderCountRBSP() noexcept = default;
  explicit constexpr FrameOrderCountRBSP(std::uint16_t frm_order_cnt_lsb) noexcept;

  [[nodiscard]] constexpr auto frm_order_cnt_lsb() const noexcept;

  constexpr auto frm_order_cnt_lsb(uint16_t value) noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const FrameOrderCountRBSP &x) -> std::ostream &;

  constexpr auto operator==(const FrameOrderCountRBSP &other) const noexcept;
  constexpr auto operator!=(const FrameOrderCountRBSP &other) const noexcept;

  static auto decodeFrom(std::istream &stream, const AtlasAdaptationParameterSetRBSP &aaps)
      -> FrameOrderCountRBSP;
  static auto decodeFrom(std::istream &stream, const AtlasSequenceParameterSetRBSP &asps)
      -> FrameOrderCountRBSP;

  void encodeTo(std::ostream &stream, const AtlasAdaptationParameterSetRBSP &aaps) const;
  void encodeTo(std::ostream &stream, const AtlasSequenceParameterSetRBSP &asps) const;

private:
  std::uint16_t m_frm_order_cnt_lsb{};
};
} // namespace TMIV::MivBitstream

#include "FrameOrderCountRBSP.hpp"

#endif
