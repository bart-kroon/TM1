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

#ifndef TMIV_MIVBITSTREAM_V3CUNIT_H
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto V3cUnitHeader::vps() noexcept -> V3cUnitHeader {
  static_assert(V3cUnitHeader{}.vuh_unit_type() == VuhUnitType::V3C_VPS);
  return {};
}

constexpr auto V3cUnitHeader::ad(uint8_t vuh_v3c_parameter_set_id, AtlasId atlas_id) noexcept
    -> V3cUnitHeader {
  auto result = V3cUnitHeader{};
  result.m_vuh_unit_type = VuhUnitType::V3C_AD;
  result.m_vuh_v3c_parameter_set_id = vuh_v3c_parameter_set_id;
  result.m_vuh_atlas_id = atlas_id;
  return result;
}

constexpr auto V3cUnitHeader::ovd(uint8_t vuh_v3c_parameter_set_id, AtlasId atlas_id) noexcept
    -> V3cUnitHeader {
  auto result = V3cUnitHeader{};
  result.m_vuh_unit_type = VuhUnitType::V3C_OVD;
  result.m_vuh_v3c_parameter_set_id = vuh_v3c_parameter_set_id;
  result.m_vuh_atlas_id = atlas_id;
  return result;
}

constexpr auto V3cUnitHeader::gvd(uint8_t vuh_v3c_parameter_set_id, AtlasId atlas_id,
                                  uint8_t vuh_map_index, bool vuh_auxiliary_video_flag) noexcept
    -> V3cUnitHeader {
  auto result = V3cUnitHeader{};
  result.m_vuh_unit_type = VuhUnitType::V3C_GVD;
  result.m_vuh_v3c_parameter_set_id = vuh_v3c_parameter_set_id;
  result.m_vuh_atlas_id = atlas_id;
  result.m_vuh_map_index = vuh_map_index;
  result.m_vuh_auxiliary_video_flag = vuh_auxiliary_video_flag;
  return result;
}

constexpr auto V3cUnitHeader::avd(uint8_t vuh_v3c_parameter_set_id, AtlasId atlas_id,
                                  uint8_t vuh_attribute_index,
                                  uint8_t vuh_attribute_partition_index, uint8_t vuh_map_index,
                                  bool vuh_auxiliary_video_flag) noexcept -> V3cUnitHeader {
  auto result = V3cUnitHeader{};
  result.m_vuh_unit_type = VuhUnitType::V3C_AVD;
  result.m_vuh_v3c_parameter_set_id = vuh_v3c_parameter_set_id;
  result.m_vuh_atlas_id = atlas_id;
  result.m_vuh_attribute_index = vuh_attribute_index;
  result.m_vuh_attribute_partition_index = vuh_attribute_partition_index;
  result.m_vuh_map_index = vuh_map_index;
  result.m_vuh_auxiliary_video_flag = vuh_auxiliary_video_flag;
  return result;
}

constexpr auto V3cUnitHeader::pvd(uint8_t vuh_v3c_parameter_set_id, AtlasId atlas_id) noexcept
    -> V3cUnitHeader {
  auto result = V3cUnitHeader{};
  result.m_vuh_unit_type = VuhUnitType::V3C_PVD;
  result.m_vuh_v3c_parameter_set_id = vuh_v3c_parameter_set_id;
  result.m_vuh_atlas_id = atlas_id;
  return result;
}

constexpr auto V3cUnitHeader::cad(uint8_t vuh_v3c_parameter_set_id) noexcept -> V3cUnitHeader {
  auto result = V3cUnitHeader{};
  result.m_vuh_unit_type = VuhUnitType::V3C_CAD;
  result.m_vuh_v3c_parameter_set_id = vuh_v3c_parameter_set_id;
  return result;
}

constexpr auto V3cUnitHeader::vuh_v3c_parameter_set_id() const noexcept -> uint8_t {
  PRECONDITION(m_vuh_unit_type == VuhUnitType::V3C_AVD || m_vuh_unit_type == VuhUnitType::V3C_GVD ||
               m_vuh_unit_type == VuhUnitType::V3C_OVD || m_vuh_unit_type == VuhUnitType::V3C_AD ||
               m_vuh_unit_type == VuhUnitType::V3C_CAD || m_vuh_unit_type == VuhUnitType::V3C_PVD);
  return m_vuh_v3c_parameter_set_id;
}

constexpr auto V3cUnitHeader::vuh_atlas_id() const noexcept -> AtlasId {
  PRECONDITION(m_vuh_unit_type == VuhUnitType::V3C_AVD || m_vuh_unit_type == VuhUnitType::V3C_GVD ||
               m_vuh_unit_type == VuhUnitType::V3C_OVD || m_vuh_unit_type == VuhUnitType::V3C_AD ||
               m_vuh_unit_type == VuhUnitType::V3C_PVD);
  return m_vuh_atlas_id;
}

constexpr auto V3cUnitHeader::vuh_attribute_index() const noexcept -> uint8_t {
  PRECONDITION(m_vuh_unit_type == VuhUnitType::V3C_AVD);
  return m_vuh_attribute_index;
}

constexpr auto V3cUnitHeader::vuh_attribute_partition_index() const noexcept -> uint8_t {
  PRECONDITION(m_vuh_unit_type == VuhUnitType::V3C_AVD);
  return m_vuh_attribute_partition_index;
}

constexpr auto V3cUnitHeader::vuh_map_index() const noexcept -> uint8_t {
  PRECONDITION(m_vuh_unit_type == VuhUnitType::V3C_AVD || m_vuh_unit_type == VuhUnitType::V3C_GVD);
  return m_vuh_map_index;
}

constexpr auto V3cUnitHeader::vuh_auxiliary_video_flag() const noexcept -> bool {
  PRECONDITION(m_vuh_unit_type == VuhUnitType::V3C_AVD || m_vuh_unit_type == VuhUnitType::V3C_GVD);
  return m_vuh_auxiliary_video_flag;
}

constexpr auto V3cUnitHeader::operator==(const V3cUnitHeader &other) const noexcept -> bool {
  if (vuh_unit_type() != other.vuh_unit_type()) {
    return false;
  }
  if (vuh_unit_type() == VuhUnitType::V3C_VPS) {
    return true;
  }
  if (vuh_v3c_parameter_set_id() != other.vuh_v3c_parameter_set_id()) {
    return false;
  }
  if (vuh_unit_type() == VuhUnitType::V3C_CAD) {
    return true;
  }
  if (vuh_atlas_id() != other.vuh_atlas_id()) {
    return false;
  }
  if (vuh_unit_type() == VuhUnitType::V3C_OVD || vuh_unit_type() == VuhUnitType::V3C_AD ||
      vuh_unit_type() == VuhUnitType::V3C_PVD) {
    return true;
  }
  if (vuh_map_index() != other.vuh_map_index() ||
      vuh_auxiliary_video_flag() != other.vuh_auxiliary_video_flag()) {
    return false;
  }
  if (vuh_unit_type() == VuhUnitType::V3C_GVD) {
    return true;
  }
  if (vuh_unit_type() == VuhUnitType::V3C_AVD) {
    return vuh_attribute_index() == other.vuh_attribute_index() &&
           vuh_attribute_partition_index() == other.vuh_attribute_partition_index();
  }
  UNREACHABLE;
}

constexpr auto V3cUnitHeader::operator!=(const V3cUnitHeader &other) const noexcept -> bool {
  return !operator==(other);
}
} // namespace TMIV::MivBitstream
