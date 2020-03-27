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
#define _TMIV_MIVBITSTREAM_VPCCUNIT_H_

#include <TMIV/MivBitstream/AtlasSubBitstream.h>
#include <TMIV/MivBitstream/VideoSubBitstream.h>
#include <TMIV/MivBitstream/VpccParameterSet.h>

#include <cstdint>
#include <cstdlib>
#include <iosfwd>
#include <string>
#include <variant>

namespace TMIV::MivBitstream {
enum class VuhUnitType : std::uint8_t { VPCC_VPS, VPCC_AD, VPCC_OVD, VPCC_GVD, VPCC_AVD };

auto operator<<(std::ostream &stream, const VuhUnitType x) -> std::ostream &;

constexpr uint8_t specialAtlasId = 0x3F;

// 23090-5: vpcc_unit_header()
class VpccUnitHeader {
public:
  explicit VpccUnitHeader(VuhUnitType vuh_unit_type) : m_vuh_unit_type{vuh_unit_type} {}

  [[nodiscard]] constexpr auto vuh_unit_type() const noexcept { return m_vuh_unit_type; }

  [[nodiscard]] auto vuh_vpcc_parameter_set_id() const noexcept -> std::uint8_t;
  [[nodiscard]] auto vuh_atlas_id() const noexcept -> std::uint8_t;
  [[nodiscard]] auto vuh_attribute_index() const noexcept -> std::uint8_t;
  [[nodiscard]] auto vuh_attribute_dimension_index() const noexcept -> std::uint8_t;
  [[nodiscard]] auto vuh_map_index() const noexcept -> std::uint8_t;
  [[nodiscard]] auto vuh_raw_video_flag() const noexcept -> bool;

  auto vuh_vpcc_parameter_set_id(const std::uint8_t value) noexcept -> VpccUnitHeader &;
  auto vuh_atlas_id(const std::uint8_t value) noexcept -> VpccUnitHeader &;
  auto vuh_attribute_index(const std::uint8_t value) noexcept -> VpccUnitHeader &;
  auto vuh_attribute_dimension_index(const std::uint8_t value) noexcept -> VpccUnitHeader &;
  auto vuh_map_index(const std::uint8_t value) noexcept -> VpccUnitHeader &;
  auto vuh_raw_video_flag(const bool value) noexcept -> VpccUnitHeader &;

  friend auto operator<<(std::ostream &stream, const VpccUnitHeader &x) -> std::ostream &;

  auto operator==(const VpccUnitHeader &other) const noexcept -> bool;
  auto operator!=(const VpccUnitHeader &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream, const std::vector<VpccParameterSet> &vpses)
      -> VpccUnitHeader;

  void encodeTo(std::ostream &stream, const std::vector<VpccParameterSet> &vpses) const;

private:
  const VuhUnitType m_vuh_unit_type;
  std::uint8_t m_vuh_vpcc_parameter_set_id{};
  std::uint8_t m_vuh_atlas_id{};
  std::uint8_t m_vuh_attribute_index{};
  std::uint8_t m_vuh_attribute_dimension_index{};
  std::uint8_t m_vuh_map_index{};
  bool m_vuh_raw_video_flag{};
};

// 23090-5: vpcc_payload()
class VpccPayload {
public:
  using Payload =
      std::variant<std::monostate, VpccParameterSet, AtlasSubBitstream, VideoSubBitstream>;

  template <typename Value>
  constexpr explicit VpccPayload(Value &&value) : m_payload{std::forward<Value>(value)} {}

  [[nodiscard]] constexpr auto payload() const noexcept -> auto & { return m_payload; }

  [[nodiscard]] auto vpcc_parameter_set() const noexcept -> const VpccParameterSet &;
  [[nodiscard]] auto atlas_sub_bitstream() const noexcept -> const AtlasSubBitstream &;
  [[nodiscard]] auto video_sub_bitstream() const noexcept -> const VideoSubBitstream &;

  friend auto operator<<(std::ostream &stream, const VpccPayload &x) -> std::ostream &;

  auto operator==(const VpccPayload &other) const noexcept -> bool;
  auto operator!=(const VpccPayload &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream, const VpccUnitHeader &vuh) -> VpccPayload;

  void encodeTo(std::ostream &stream, const VpccUnitHeader &vuh) const;

private:
  const Payload m_payload;
};

// 23090-5: vpcc_unit(NumBytesInVPCCUnit)
class VpccUnit {
public:
  template <typename Payload>
  VpccUnit(const VpccUnitHeader &vpcc_unit_header, Payload &&payload)
      : m_vpcc_unit_header{vpcc_unit_header}, m_vpcc_payload{std::forward<Payload>(payload)} {}

  [[nodiscard]] constexpr auto vpcc_unit_header() const noexcept -> auto & {
    return m_vpcc_unit_header;
  }
  [[nodiscard]] constexpr auto vpcc_payload() const noexcept -> auto & { return m_vpcc_payload; }

  friend auto operator<<(std::ostream &stream, const VpccUnit &x) -> std::ostream &;

  auto operator==(const VpccUnit &other) const noexcept -> bool;
  auto operator!=(const VpccUnit &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream, const std::vector<VpccParameterSet> &vpses,
                         std::size_t numBytesInVPCCUnit) -> VpccUnit;

  auto encodeTo(std::ostream &stream, const std::vector<VpccParameterSet> &vpses) const
      -> std::size_t;

private:
  VpccUnitHeader m_vpcc_unit_header;
  VpccPayload m_vpcc_payload;
};
} // namespace TMIV::MivBitstream

#endif
