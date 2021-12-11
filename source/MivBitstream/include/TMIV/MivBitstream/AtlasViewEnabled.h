/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#ifndef TMIV_MIVBITSTREAM_ATLASVIEWENABLED_H
#define TMIV_MIVBITSTREAM_ATLASVIEWENABLED_H

#include <TMIV/Common/Bitstream.h>

#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
// 23090-12: atlas_view_enabled( payloadSize )
class AtlasViewEnabled {
public:
  [[nodiscard]] auto ave_cancel_flag() const noexcept -> bool;
  [[nodiscard]] auto ave_persistence_flag() const -> bool;
  [[nodiscard]] auto ave_atlas_count_minus1() const -> uint8_t;
  [[nodiscard]] auto ave_num_views_minus1() const -> uint16_t;
  [[nodiscard]] auto ave_atlas_id(uint8_t atlasIdx) const -> uint8_t;
  [[nodiscard]] auto ave_view_enabled_in_atlas_flag(uint8_t atlasId, uint16_t viewIdx) const
      -> bool;
  [[nodiscard]] auto ave_view_complete_in_atlas_flag(uint8_t atlasId, uint16_t viewIdx) const
      -> bool;

  auto ave_cancel_flag(bool value) noexcept -> AtlasViewEnabled &;
  auto ave_persistence_flag(bool value) noexcept -> AtlasViewEnabled &;
  auto ave_atlas_count_minus1(uint8_t value) noexcept -> AtlasViewEnabled &;
  auto ave_num_views_minus1(uint16_t value) noexcept -> AtlasViewEnabled &;
  auto ave_atlas_id(uint8_t atlasIdx, uint8_t value) -> AtlasViewEnabled &;
  auto ave_view_enabled_in_atlas_flag(uint8_t atlasId, uint16_t viewIdx, bool value)
      -> AtlasViewEnabled &;
  auto ave_view_complete_in_atlas_flag(uint8_t atlasId, uint16_t viewIdx, bool value)
      -> AtlasViewEnabled &;

  friend auto operator<<(std::ostream &stream, const AtlasViewEnabled &x) -> std::ostream &;

  auto operator==(const AtlasViewEnabled &other) const -> bool;
  auto operator!=(const AtlasViewEnabled &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> AtlasViewEnabled;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_ave_cancel_flag{true};
  std::optional<bool> m_ave_persistence_flag{};
  std::optional<uint8_t> m_ave_atlas_count_minus1{};
  std::optional<uint16_t> m_ave_num_views_minus1{};
  std::optional<std::vector<uint8_t>> m_ave_atlas_id{};
  std::optional<std::vector<std::vector<bool>>> m_ave_view_enabled_in_atlas_flag{};
  std::optional<std::vector<std::vector<bool>>> m_ave_view_complete_in_atlas_flag{};
};
} // namespace TMIV::MivBitstream

#endif
