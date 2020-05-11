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

#ifndef _TMIV_MIVBITSTREAM_V3CUNIT_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto V3C_VPS ::operator==(const V3C_VPS &other) const noexcept -> bool { return true; }
constexpr auto V3C_VPS ::operator!=(const V3C_VPS &other) const noexcept -> bool { return false; }

//  std::uint8_t vuh_v3c_parameter_set_id;
// std::uint8_t vuh_atlas_id;

constexpr auto V3C_VPS ::operator==(const V3C_OVD &other) const noexcept -> bool;
constexpr auto V3C_VPS ::operator!=(const V3C_OVD &other) const noexcept -> bool;

//  std::uint8_t m_vuh_map_index;
//  std::uint8_t m_vuh_raw_video_flag;

constexpr auto V3C_OVD::operator==(const V3C_OVD &other) const noexcept -> bool;
constexpr auto V3C_OVD::operator!=(const V3C_OVD &other) const noexcept -> bool;

struct V3C_AVD : public V3C_GVD {
  std::uint8_t vuh_attribute_index;
  std::uint8_t vuh_attribute_dimension_index;

  friend auto operator<<(std::ostream &stream, const V3C_AVD &x) -> std::ostream &;

  constexpr auto operator==(const V3C_AVD &other) const noexcept -> bool;
  constexpr auto operator!=(const V3C_AVD &other) const noexcept -> bool;
};

using VuhUnitType = std::variant<V3C_VPS, V3C_AD, V3C_OVD, V3C_GVD, V3C_AVD>;

// 23090-5: v3c_unit_header()
class V3cUnitHeader {
public:
  explicit V3cUnitHeader(VuhUnitType vuh_unit_type) : m_vuh_unit_type{vuh_unit_type} {}

  constexpr auto vuh_unit_type() const noexcept { return m_vuh_unit_type; }
  auto VPS() const -> V3C_VPS;
  auto AD() const -> V3C_AD;
  auto OVD() const -> V3C_OVD;
  auto GVD() const -> V3C_GVD;
  auto AVD() const -> V3C_AVD;

  friend auto operator<<(std::ostream &stream, const V3cUnitHeader &x) -> std::ostream &;

  auto operator==(const V3cUnitHeader &other) const noexcept -> bool;
  auto operator!=(const V3cUnitHeader &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> V3cUnitHeader;

  void encodeTo(std::ostream &stream) const;

private:
  VuhUnitType m_vuh_unit_type;
};

// 23090-5: v3c_payload()
class V3cPayload {
public:
  friend auto operator<<(std::ostream &stream, const V3cPayload &x) -> std::ostream &;

  constexpr auto operator==(const V3cPayload &other) const noexcept -> bool;
  constexpr auto operator!=(const V3cPayload &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> V3cPayload;
  void encodeTo(std::ostream &stream) const;
};

// 23090-5: v3c_unit(NumBytesInV3CUnit)
class V3cUnit {
public:
  V3cUnit(const V3cUnitHeader &vppc_unit_header, const V3cPayload &v3c_payload);

  constexpr auto &vppc_unit_header() const noexcept { return m_v3c_unit_header; }
  constexpr auto &v3c_payload() const noexcept { return m_v3c_payload; }

  friend auto operator<<(std::ostream &stream, const V3cUnit &x) -> std::ostream &;

  constexpr auto operator==(const V3cUnit &other) const noexcept -> bool {
    return m_v3c_unit_header == other.m_v3c_unit_header && m_v3c_payload == other.m_v3c_payload;
  }

  constexpr auto operator!=(const V3cUnit &other) const noexcept -> bool {
    return !operator==(other);
  }

  static auto decodeFrom(std::istream &stream, std::size_t numBytesInV3CUnit) -> V3cUnit;
  void encodeTo(std::ostream &stream, std::size_t numBytesInV3CUnit) const;

private:
  V3cUnitHeader m_v3c_unit_header;
  V3cPayload m_v3c_payload;
};
} // namespace TMIV::MivBitstream

#endif
