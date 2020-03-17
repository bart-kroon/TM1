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

#ifndef _TMIV_MIVBITSTREAM_VPCCUNIT_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto VPCC_VPS ::operator==(const VPCC_VPS &other) const noexcept -> bool { return true; }
constexpr auto VPCC_VPS ::operator!=(const VPCC_VPS &other) const noexcept -> bool { return false; }

//  std::uint8_t vuh_vpcc_parameter_set_id;
// std::uint8_t vuh_atlas_id;

constexpr auto VPCC_VPS ::operator==(const VPCC_OVD &other) const noexcept -> bool;
constexpr auto VPCC_VPS ::operator!=(const VPCC_OVD &other) const noexcept -> bool;

//  std::uint8_t m_vuh_map_index;
//  std::uint8_t m_vuh_raw_video_flag;

constexpr auto VPCC_OVD::operator==(const VPCC_OVD &other) const noexcept -> bool;
constexpr auto VPCC_OVD::operator!=(const VPCC_OVD &other) const noexcept -> bool;

struct VPCC_AVD : public VPCC_GVD {
  std::uint8_t vuh_attribute_index;
  std::uint8_t vuh_attribute_dimension_index;

  friend auto operator<<(std::ostream &stream, const VPCC_AVD &x) -> std::ostream &;

  constexpr auto operator==(const VPCC_AVD &other) const noexcept -> bool;
  constexpr auto operator!=(const VPCC_AVD &other) const noexcept -> bool;
};

using VuhUnitType = std::variant<VPCC_VPS, VPCC_AD, VPCC_OVD, VPCC_GVD, VPCC_AVD>;

// 23090-5: vpcc_unit_header()
class VpccUnitHeader {
public:
  explicit VpccUnitHeader(VuhUnitType vuh_unit_type) : m_vuh_unit_type{vuh_unit_type} {}

  constexpr auto vuh_unit_type() const noexcept { return m_vuh_unit_type; }
  auto VPS() const -> VPCC_VPS;
  auto AD() const -> VPCC_AD;
  auto OVD() const -> VPCC_OVD;
  auto GVD() const -> VPCC_GVD;
  auto AVD() const -> VPCC_AVD;

  friend auto operator<<(std::ostream &stream, const VpccUnitHeader &x) -> std::ostream &;

  auto operator==(const VpccUnitHeader &other) const noexcept -> bool;
  auto operator!=(const VpccUnitHeader &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> VpccUnitHeader;

  void encodeTo(std::ostream &stream) const;

private:
  VuhUnitType m_vuh_unit_type;
};

// 23090-5: vpcc_payload()
class VpccPayload {
public:
  friend auto operator<<(std::ostream &stream, const VpccPayload &x) -> std::ostream &;

  constexpr auto operator==(const VpccPayload &other) const noexcept -> bool;
  constexpr auto operator!=(const VpccPayload &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> VpccPayload;
  void encodeTo(std::ostream &stream) const;
};

// 23090-5: vpcc_unit(NumBytesInVPCCUnit)
class VpccUnit {
public:
  VpccUnit(const VpccUnitHeader &vppc_unit_header, const VpccPayload &vpcc_payload);

  constexpr auto &vppc_unit_header() const noexcept { return m_vpcc_unit_header; }
  constexpr auto &vpcc_payload() const noexcept { return m_vpcc_payload; }

  friend auto operator<<(std::ostream &stream, const VpccUnit &x) -> std::ostream &;

  constexpr auto operator==(const VpccUnit &other) const noexcept -> bool {
    return m_vpcc_unit_header == other.m_vpcc_unit_header && m_vpcc_payload == other.m_vpcc_payload;
  }

  constexpr auto operator!=(const VpccUnit &other) const noexcept -> bool {
    return !operator==(other);
  }

  static auto decodeFrom(std::istream &stream, std::size_t numBytesInVPCCUnit) -> VpccUnit;
  void encodeTo(std::ostream &stream, std::size_t numBytesInVPCCUnit) const;

private:
  VpccUnitHeader m_vpcc_unit_header;
  VpccPayload m_vpcc_payload;
};
} // namespace TMIV::MivBitstream

#endif
