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

#ifndef _TMIV_MIVBITSTREAM_RECOMMENDEDVIEWPORT_H_
#define _TMIV_MIVBITSTREAM_RECOMMENDEDVIEWPORT_H_

#include <TMIV/Common/Bitstream.h>

#include <vector>

namespace TMIV::MivBitstream {
//struct RecommendedViewportParams {};

// 23090-12: rec_viewport()
class RecommendedViewport {
public:
  RecommendedViewport() = default;

  auto rec_viewport_sei_size() const noexcept -> std::size_t;

  friend auto operator<<(std::ostream &stream, const RecommendedViewport &x) -> std::ostream &;

  auto operator==(const RecommendedViewport &other) const noexcept -> bool;
  auto operator!=(const RecommendedViewport &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> RecommendedViewport;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  //RecommendedViewportParams m_RecommendedViewportParams;
  std::uint16_t m_rec_viewport_id{};
  bool m_rec_viewport_cancel_flag{};
  bool m_rec_viewport_persistence_flag{};
  bool m_rec_viewport_center_view_flag{};
  bool m_rec_viewport_left_view_flag{};
  float m_rec_viewport_pos_x{};
  float m_rec_viewport_pos_y{};
  float m_rec_viewport_pos_z{};
  float m_rec_viewport_quat_x{};
  float m_rec_viewport_quat_y{};
  float m_rec_viewport_quat_z{};
  float m_rec_viewport_hor_range{};
  float m_rec_viewport_ver_range{};
};
} // namespace TMIV::MivBitstream

#endif
