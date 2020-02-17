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

#ifndef _TMIV_MIVBITSTREAM_ATLASTILEGROUPLAYERRBSP_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto AtlasTileGroupHeader::atgh_atlas_frame_parameter_set_id() const noexcept {
  return m_atgh_atlas_frame_parameter_set_id;
}

constexpr auto AtlasTileGroupHeader::atgh_address() const noexcept { return m_atgh_address; }

constexpr auto AtlasTileGroupHeader::atgh_type() const noexcept { return m_atgh_type; }

constexpr auto AtlasTileGroupHeader::atgh_atlas_frm_order_cnt_lsb() const noexcept {
  return m_atgh_atlas_frm_order_cnt_lsb;
}

constexpr auto &
AtlasTileGroupHeader::atgh_atlas_frame_parameter_set_id(const std::uint8_t value) noexcept {
  m_atgh_atlas_frame_parameter_set_id = value;
  return *this;
}

constexpr auto &
AtlasTileGroupHeader::atgh_adaptation_parameter_set_id(const std::uint8_t value) noexcept {
  m_atgh_adaptation_parameter_set_id = value;
  return *this;
}

constexpr auto &AtlasTileGroupHeader::atgh_address(const std::uint8_t value) noexcept {
  m_atgh_address = value;
  return *this;
}

constexpr auto &AtlasTileGroupHeader::atgh_type(const AtghType value) noexcept {
  m_atgh_type = value;
  return *this;
}

constexpr auto &
AtlasTileGroupHeader::atgh_atlas_frm_order_cnt_lsb(const std::uint8_t value) noexcept {
  m_atgh_atlas_frm_order_cnt_lsb = value;
  return *this;
}

constexpr auto AtlasTileGroupHeader::operator==(const AtlasTileGroupHeader &other) const noexcept {
  if (atgh_atlas_frame_parameter_set_id() != other.atgh_atlas_frame_parameter_set_id() ||
      atgh_address() != other.atgh_address() || atgh_type() != other.atgh_type() ||
      atgh_atlas_frm_order_cnt_lsb() != other.atgh_atlas_frm_order_cnt_lsb()) {
    return false;
  }
  if (atgh_type() == AtghType::SKIP_TILE_GRP) {
    return true;
  }
  return atgh_patch_size_x_info_quantizer() == other.atgh_patch_size_x_info_quantizer() &&
         atgh_patch_size_y_info_quantizer() == other.atgh_patch_size_y_info_quantizer();
}

constexpr auto AtlasTileGroupHeader::operator!=(const AtlasTileGroupHeader &other) const noexcept {
  return !operator==(other);
}

constexpr auto SkipPatchDataUnit::operator==(const SkipPatchDataUnit & /* other */) const noexcept {
  return true;
}
constexpr auto SkipPatchDataUnit::operator!=(const SkipPatchDataUnit & /* other */) const noexcept {
  return false;
}

inline auto SkipPatchDataUnit::decodeFrom(Common::InputBitstream & /* bitstream */)
    -> SkipPatchDataUnit {
  return {};
}

inline void SkipPatchDataUnit::encodeTo(Common::OutputBitstream & /* bitstream */) const {}

constexpr auto PatchDataUnit::pdu_2d_pos_x() const noexcept { return m_pdu_2d_pos_x; }

constexpr auto PatchDataUnit::pdu_2d_pos_y() const noexcept { return m_pdu_2d_pos_y; }

constexpr auto PatchDataUnit::pdu_2d_delta_size_x() const noexcept { return m_pdu_2d_delta_size_x; }

constexpr auto PatchDataUnit::pdu_2d_delta_size_y() const noexcept { return m_pdu_2d_delta_size_y; }

constexpr auto PatchDataUnit::pdu_view_pos_x() const noexcept { return m_pdu_view_pos_x; }

constexpr auto PatchDataUnit::pdu_view_pos_y() const noexcept { return m_pdu_view_pos_y; }

constexpr auto PatchDataUnit::pdu_depth_start() const noexcept { return m_pdu_depth_start; }

constexpr auto PatchDataUnit::pdu_projection_id() const noexcept { return m_pdu_projection_id; }

constexpr auto PatchDataUnit::pdu_orientation_index() const noexcept {
  return m_pdu_orientation_index;
}

constexpr auto &PatchDataUnit::pdu_2d_pos_x(const std::uint32_t value) noexcept {
  m_pdu_2d_pos_x = value;
  return *this;
}

constexpr auto &PatchDataUnit::pdu_2d_pos_y(const std::uint32_t value) noexcept {
  m_pdu_2d_pos_y = value;
  return *this;
}

constexpr auto &PatchDataUnit::pdu_2d_delta_size_x(const std::int32_t value) noexcept {
  m_pdu_2d_delta_size_x = value;
  return *this;
}

constexpr auto &PatchDataUnit::pdu_2d_delta_size_y(const std::int32_t value) noexcept {
  m_pdu_2d_delta_size_y = value;
  return *this;
}

constexpr auto &PatchDataUnit::pdu_view_pos_x(const std::uint32_t value) noexcept {
  m_pdu_view_pos_x = value;
  return *this;
}

constexpr auto &PatchDataUnit::pdu_view_pos_y(const std::uint32_t value) noexcept {
  m_pdu_view_pos_y = value;
  return *this;
}

constexpr auto &PatchDataUnit::pdu_depth_start(const std::uint32_t value) noexcept {
  m_pdu_depth_start = value;
  return *this;
}

constexpr auto &PatchDataUnit::pdu_depth_end(const std::uint32_t value) noexcept {
  m_pdu_depth_end = value;
  return *this;
}

constexpr auto &PatchDataUnit::pdu_projection_id(const std::uint16_t value) noexcept {
  m_pdu_projection_id = value;
  return *this;
}

constexpr auto &
PatchDataUnit::pdu_orientation_index(const FlexiblePatchOrientation value) noexcept {
  m_pdu_orientation_index = value;
  return *this;
}

constexpr auto &PatchDataUnit::pdu_entity_id(const unsigned value) noexcept {
  m_pdu_entity_id = value;
  return *this;
}

constexpr auto PatchDataUnit::operator==(const PatchDataUnit &other) const noexcept {
  return pdu_2d_pos_x() == other.pdu_2d_pos_x() && pdu_2d_pos_y() == other.pdu_2d_pos_y() &&
         pdu_2d_delta_size_x() == other.pdu_2d_delta_size_x() &&
         pdu_2d_delta_size_y() == other.pdu_2d_delta_size_y() &&
         pdu_view_pos_x() == other.pdu_view_pos_x() && pdu_view_pos_y() == other.pdu_view_pos_y() &&
         pdu_depth_start() == other.pdu_depth_start() && m_pdu_depth_end == other.m_pdu_depth_end &&
         pdu_projection_id() == other.pdu_projection_id() &&
         pdu_orientation_index() == other.pdu_orientation_index();
}

constexpr auto PatchDataUnit::operator!=(const PatchDataUnit &other) const noexcept {
  return !operator==(other);
}

constexpr auto &PatchInformationData::data() const noexcept { return m_data; }

template <typename Visitor> void AtlasTileGroupDataUnit::visit(Visitor &&visitor) const {
  for (std::size_t p = 0; p < m_vector.size(); ++p) {
    visitor(p, m_vector[p].first, m_vector[p].second);
  }
}

constexpr auto AtlasTileGroupLayerRBSP::atlas_tile_group_header() const noexcept
    -> const AtlasTileGroupHeader & {
  return m_atlas_tile_group_header;
}

constexpr auto AtlasTileGroupLayerRBSP::
operator==(const AtlasTileGroupLayerRBSP & /* other */) const noexcept -> bool {
  return true;
}

constexpr auto AtlasTileGroupLayerRBSP::
operator!=(const AtlasTileGroupLayerRBSP & /* other */) const noexcept -> bool {
  return false;
}
} // namespace TMIV::MivBitstream
