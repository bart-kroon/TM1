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

#include <TMIV/MivBitstream/AtlasViewEnabled.h>

namespace TMIV::MivBitstream {
auto AtlasViewEnabled::ave_cancel_flag() const noexcept -> bool { return m_ave_cancel_flag; }

auto AtlasViewEnabled::ave_persistence_flag() const -> bool {
  VERIFY_MIVBITSTREAM(m_ave_persistence_flag.has_value());
  return *m_ave_persistence_flag;
}

auto AtlasViewEnabled::ave_atlas_count_minus1() const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_ave_atlas_count_minus1.has_value());
  return *m_ave_atlas_count_minus1;
}

auto AtlasViewEnabled::ave_num_views_minus1() const -> uint16_t {
  VERIFY_MIVBITSTREAM(m_ave_num_views_minus1.has_value());
  return *m_ave_num_views_minus1;
}

auto AtlasViewEnabled::ave_atlas_id(uint8_t atlasIdx) const -> uint8_t {
  PRECONDITION(atlasIdx <= ave_atlas_count_minus1());
  VERIFY_MIVBITSTREAM(m_ave_atlas_id.has_value());
  return m_ave_atlas_id.value().at(atlasIdx);
}

auto AtlasViewEnabled::ave_view_enabled_in_atlas_flag(uint8_t atlasId, uint16_t viewIdx) const
    -> bool {
  PRECONDITION(viewIdx <= ave_num_views_minus1());
  VERIFY_MIVBITSTREAM(m_ave_view_enabled_in_atlas_flag.has_value());
  VERIFY_MIVBITSTREAM(atlasId < m_ave_view_enabled_in_atlas_flag.value().size());
  return m_ave_view_enabled_in_atlas_flag.value().at(atlasId).at(viewIdx);
}

auto AtlasViewEnabled::ave_view_complete_in_atlas_flag(uint8_t atlasId, uint16_t viewIdx) const
    -> bool {
  PRECONDITION(viewIdx <= ave_num_views_minus1());
  VERIFY_MIVBITSTREAM(ave_view_enabled_in_atlas_flag(atlasId, viewIdx));
  VERIFY_MIVBITSTREAM(m_ave_view_complete_in_atlas_flag.has_value());
  VERIFY_MIVBITSTREAM(atlasId < m_ave_view_complete_in_atlas_flag.value().size());
  return m_ave_view_complete_in_atlas_flag.value().at(atlasId).at(viewIdx);
}

auto AtlasViewEnabled::ave_cancel_flag(bool value) noexcept -> AtlasViewEnabled & {
  m_ave_cancel_flag = value;
  return *this;
}

auto AtlasViewEnabled::ave_persistence_flag(bool value) noexcept -> AtlasViewEnabled & {
  m_ave_persistence_flag = value;
  return *this;
}

auto AtlasViewEnabled::ave_atlas_count_minus1(uint8_t value) noexcept -> AtlasViewEnabled & {
  m_ave_atlas_count_minus1 = value;
  if (m_ave_atlas_id.has_value()) {
    m_ave_atlas_id.value().clear();
  }
  m_ave_atlas_id.emplace(value + 1);
  return *this;
}

auto AtlasViewEnabled::ave_num_views_minus1(uint16_t value) noexcept -> AtlasViewEnabled & {
  m_ave_num_views_minus1 = value;
  return *this;
}

auto AtlasViewEnabled::ave_atlas_id(uint8_t atlasIdx, uint8_t value) -> AtlasViewEnabled & {
  PRECONDITION(atlasIdx <= ave_atlas_count_minus1());
  VERIFY_MIVBITSTREAM(m_ave_atlas_id.has_value());
  m_ave_atlas_id.value().at(atlasIdx) = value;
  return *this;
}

auto AtlasViewEnabled::ave_view_enabled_in_atlas_flag(uint8_t atlasId, uint16_t viewIdx, bool value)
    -> AtlasViewEnabled & {
  PRECONDITION(viewIdx <= ave_num_views_minus1());
  if (!m_ave_view_enabled_in_atlas_flag.has_value()) {
    m_ave_view_enabled_in_atlas_flag.emplace(ave_atlas_count_minus1() + 1);
  }
  while (m_ave_view_enabled_in_atlas_flag.value().at(atlasId).size() <
         static_cast<uint32_t>(ave_num_views_minus1() + static_cast<uint16_t>(1))) {
    m_ave_view_enabled_in_atlas_flag.value().at(atlasId).push_back(false);
  }
  m_ave_view_enabled_in_atlas_flag.value().at(atlasId).at(viewIdx) = value;
  return *this;
}

auto AtlasViewEnabled::ave_view_complete_in_atlas_flag(uint8_t atlasId, uint16_t viewIdx,
                                                       bool value) -> AtlasViewEnabled & {
  PRECONDITION(ave_view_enabled_in_atlas_flag(atlasId, viewIdx));
  if (!m_ave_view_complete_in_atlas_flag.has_value()) {
    m_ave_view_complete_in_atlas_flag.emplace(ave_atlas_count_minus1() + 1);
  }
  while (m_ave_view_complete_in_atlas_flag.value().at(atlasId).size() <
         static_cast<uint32_t>(ave_num_views_minus1() + static_cast<uint16_t>(1))) {
    m_ave_view_complete_in_atlas_flag.value().at(atlasId).push_back(false);
  }
  m_ave_view_complete_in_atlas_flag.value().at(atlasId).at(viewIdx) = value;
  return *this;
}

auto operator<<(std::ostream &stream, const AtlasViewEnabled &x) -> std::ostream & {
  stream << "ave_cancel_flag=" << std::boolalpha << x.ave_cancel_flag() << '\n';
  if (!x.ave_cancel_flag()) {
    stream << "ave_persistence_flag=" << std::boolalpha << x.ave_persistence_flag() << '\n';
    stream << "ave_atlas_count_minus1=" << static_cast<uint32_t>(x.ave_atlas_count_minus1())
           << '\n';
    stream << "ave_num_views_minus1=" << static_cast<uint32_t>(x.ave_num_views_minus1()) << '\n';
    for (uint8_t a = 0; a <= x.ave_atlas_count_minus1(); a++) {
      stream << "ave_atlas_id[ " << static_cast<uint32_t>(a)
             << " ]=" << static_cast<uint32_t>(x.ave_atlas_id(a)) << '\n';
      const auto atlasId = x.ave_atlas_id(a);
      for (uint16_t v = 0; v <= x.ave_num_views_minus1(); v++) {
        stream << "ave_view_enabled_in_atlas_flag[ " << static_cast<uint32_t>(a) << " ][ "
               << static_cast<uint32_t>(v) << " ]=" << std::boolalpha
               << x.ave_view_enabled_in_atlas_flag(atlasId, v) << '\n';
        if (x.ave_view_enabled_in_atlas_flag(atlasId, v)) {
          stream << "ave_view_complete_in_atlas_flag[ " << static_cast<uint32_t>(a) << " ][ "
                 << static_cast<uint32_t>(v) << " ]=" << std::boolalpha
                 << x.ave_view_complete_in_atlas_flag(atlasId, v) << '\n';
        }
      }
    }
  }
  return stream;
}

auto AtlasViewEnabled::operator==(const AtlasViewEnabled &other) const -> bool {
  if (ave_cancel_flag()) {
    return ave_cancel_flag() == other.ave_cancel_flag();
  }
  return ave_cancel_flag() == other.ave_cancel_flag() &&
         ave_persistence_flag() == other.ave_persistence_flag() &&
         ave_atlas_count_minus1() == other.ave_atlas_count_minus1() &&
         ave_num_views_minus1() == other.ave_num_views_minus1() &&
         ave_atlas_id(0) == other.ave_atlas_id(0) &&
         ave_view_enabled_in_atlas_flag(ave_atlas_id(0), 0) ==
             other.ave_view_enabled_in_atlas_flag(ave_atlas_id(0), 0);
}

auto AtlasViewEnabled::operator!=(const AtlasViewEnabled &other) const -> bool {
  return !operator==(other);
}

auto AtlasViewEnabled::decodeFrom(Common::InputBitstream &bitstream) -> AtlasViewEnabled {
  auto x = AtlasViewEnabled{};
  x.ave_cancel_flag(bitstream.getFlag());
  if (!x.ave_cancel_flag()) {
    x.ave_persistence_flag(bitstream.getFlag());
    x.ave_atlas_count_minus1(bitstream.readBits<uint8_t>(6));
    x.ave_num_views_minus1(bitstream.getUint16());
    for (uint8_t a = 0; a <= x.ave_atlas_count_minus1(); a++) {
      x.ave_atlas_id(a, bitstream.getUVar<uint8_t>(x.ave_atlas_count_minus1() + 1));
      const auto atlasId = x.ave_atlas_id(a);
      for (uint16_t v = 0; v <= x.ave_num_views_minus1(); v++) {
        x.ave_view_enabled_in_atlas_flag(atlasId, v, bitstream.getFlag());
        if (x.ave_view_enabled_in_atlas_flag(atlasId, v)) {
          x.ave_view_complete_in_atlas_flag(atlasId, v, bitstream.getFlag());
        }
      }
    }
  }
  return x;
}

void AtlasViewEnabled::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFlag(ave_cancel_flag());
  if (!ave_cancel_flag()) {
    bitstream.putFlag(ave_persistence_flag());
    bitstream.writeBits(ave_atlas_count_minus1(), 6);
    bitstream.putUint16(ave_num_views_minus1());
    for (uint8_t a = 0; a <= ave_atlas_count_minus1(); a++) {
      bitstream.putUVar(ave_atlas_id(a), ave_atlas_count_minus1() + 1);
      const auto atlasId = ave_atlas_id(a);
      for (uint16_t v = 0; v <= ave_num_views_minus1(); v++) {
        bitstream.putFlag(ave_view_enabled_in_atlas_flag(atlasId, v));
        if (ave_view_enabled_in_atlas_flag(atlasId, v)) {
          bitstream.putFlag(ave_view_complete_in_atlas_flag(atlasId, v));
        }
      }
    }
  }
}
} // namespace TMIV::MivBitstream
