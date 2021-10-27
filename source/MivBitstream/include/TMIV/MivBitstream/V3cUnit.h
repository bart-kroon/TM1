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

#ifndef TMIV_MIVBITSTREAM_V3CUNIT_H
#define TMIV_MIVBITSTREAM_V3CUNIT_H

#include <TMIV/MivBitstream/AtlasSubBitstream.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>
#include <TMIV/MivBitstream/VideoSubBitstream.h>

#include <cstdint>
#include <cstdlib>
#include <iosfwd>
#include <string>
#include <variant>

namespace TMIV::MivBitstream {
// 23090-5: v3c_unit_header()
class V3cUnitHeader {
public:
  [[nodiscard]] static constexpr auto vps() noexcept -> V3cUnitHeader;

  [[nodiscard]] static constexpr auto ad(uint8_t vuh_v3c_parameter_set_id,
                                         AtlasId atlas_id) noexcept -> V3cUnitHeader;

  [[nodiscard]] static constexpr auto ovd(uint8_t vuh_v3c_parameter_set_id,
                                          AtlasId atlas_id) noexcept -> V3cUnitHeader;

  [[nodiscard]] static constexpr auto gvd(uint8_t vuh_v3c_parameter_set_id, AtlasId atlas_id,
                                          uint8_t vuh_map_index = 0,
                                          bool vuh_auxiliary_video_flag = false) noexcept
      -> V3cUnitHeader;

  [[nodiscard]] static constexpr auto
  avd(uint8_t vuh_v3c_parameter_set_id, AtlasId atlas_id, uint8_t vuh_attribute_index,
      uint8_t vuh_attribute_partition_index = 0, uint8_t vuh_map_index = 0,
      bool vuh_auxiliary_video_flag = false) noexcept -> V3cUnitHeader;

  [[nodiscard]] static constexpr auto pvd(uint8_t vuh_v3c_parameter_set_id,
                                          AtlasId atlas_id) noexcept -> V3cUnitHeader;

  [[nodiscard]] static constexpr auto cad(uint8_t vuh_v3c_parameter_set_id) noexcept
      -> V3cUnitHeader;

  [[nodiscard]] constexpr auto vuh_unit_type() const noexcept { return m_vuh_unit_type; }
  [[nodiscard]] constexpr auto vuh_v3c_parameter_set_id() const noexcept -> uint8_t;
  [[nodiscard]] constexpr auto vuh_atlas_id() const noexcept -> AtlasId;
  [[nodiscard]] constexpr auto vuh_attribute_index() const noexcept -> uint8_t;
  [[nodiscard]] constexpr auto vuh_attribute_partition_index() const noexcept -> uint8_t;
  [[nodiscard]] constexpr auto vuh_map_index() const noexcept -> uint8_t;
  [[nodiscard]] constexpr auto vuh_auxiliary_video_flag() const noexcept -> bool;

  friend auto operator<<(std::ostream &stream, const V3cUnitHeader &x) -> std::ostream &;

  constexpr auto operator==(const V3cUnitHeader &other) const noexcept -> bool;
  constexpr auto operator!=(const V3cUnitHeader &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> V3cUnitHeader;

  void encodeTo(std::ostream &stream) const;

  [[nodiscard]] auto summary() const -> std::string;

private:
  VuhUnitType m_vuh_unit_type{};
  uint8_t m_vuh_v3c_parameter_set_id{};
  AtlasId m_vuh_atlas_id{};
  uint8_t m_vuh_attribute_index{};
  uint8_t m_vuh_attribute_partition_index{};
  uint8_t m_vuh_map_index{};
  bool m_vuh_auxiliary_video_flag{};
};

// 23090-5: v3c_unit_payload()
class V3cUnitPayload {
public:
  using Payload =
      std::variant<std::monostate, V3cParameterSet, AtlasSubBitstream, VideoSubBitstream>;

  // NOTE(#488): SFINAE to keep rule of zero [bugprone-forwarding-reference-overload]
  template <typename Value,
            typename = std::enable_if_t<!std::is_same_v<std::decay_t<Value>, V3cUnitPayload>>>
  constexpr explicit V3cUnitPayload(Value &&value) : m_payload{std::forward<Value>(value)} {}

  [[nodiscard]] constexpr auto payload() const noexcept -> auto & { return m_payload; }

  [[nodiscard]] auto v3c_parameter_set() const noexcept -> const V3cParameterSet &;
  [[nodiscard]] auto atlas_sub_bitstream() const noexcept -> const AtlasSubBitstream &;
  [[nodiscard]] auto video_sub_bitstream() const noexcept -> const VideoSubBitstream &;

  friend auto operator<<(std::ostream &stream, const V3cUnitPayload &x) -> std::ostream &;

  auto operator==(const V3cUnitPayload &other) const noexcept -> bool;
  auto operator!=(const V3cUnitPayload &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream, const V3cUnitHeader &vuh) -> V3cUnitPayload;

  void encodeTo(std::ostream &stream, const V3cUnitHeader &vuh) const;

private:
  const Payload m_payload;
};

// 23090-5: v3c_unit(NumBytesInV3CUnit)
class V3cUnit {
public:
  template <typename Payload>
  V3cUnit(const V3cUnitHeader &v3c_unit_header, Payload &&payload)
      : m_v3c_unit_header{v3c_unit_header}, m_v3c_unit_payload{std::forward<Payload>(payload)} {}

  [[nodiscard]] constexpr auto v3c_unit_header() const noexcept -> auto & {
    return m_v3c_unit_header;
  }
  [[nodiscard]] constexpr auto v3c_unit_payload() const noexcept -> auto & {
    return m_v3c_unit_payload;
  }

  friend auto operator<<(std::ostream &stream, const V3cUnit &x) -> std::ostream &;

  auto operator==(const V3cUnit &other) const -> bool;
  auto operator!=(const V3cUnit &other) const -> bool;

  static auto decodeFrom(std::istream &stream, size_t numBytesInV3CUnit) -> V3cUnit;

  auto encodeTo(std::ostream &stream) const -> size_t;

private:
  V3cUnitHeader m_v3c_unit_header;
  V3cUnitPayload m_v3c_unit_payload;
};
} // namespace TMIV::MivBitstream

#include "V3cUnit.hpp"

#endif
