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

#ifndef TMIV_MIVBITSTREAM_ATLASADAPTATIONPARAMETERSETRBSP_H
#define TMIV_MIVBITSTREAM_ATLASADAPTATIONPARAMETERSETRBSP_H

#include "VuiParameters.h"

#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
// 23090-5: aaps_vpcc_extension( )
//
// Limitations of this implementation:
//   * aaps_vpcc_camera_parameters_present_flag == 0
class AapsVpccExtension {
public:
  friend auto operator<<(std::ostream &stream, const AapsVpccExtension &x) -> std::ostream &;

  constexpr auto operator==(const AapsVpccExtension &other) const noexcept;
  constexpr auto operator!=(const AapsVpccExtension &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> AapsVpccExtension;

  static void encodeTo(Common::OutputBitstream &bitstream);
};

// 23090-5: atlas_adaptation_parameter_set_rbsp( )
class AtlasAdaptationParameterSetRBSP {
public:
  [[nodiscard]] constexpr auto aaps_atlas_adaptation_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto aaps_log2_max_afoc_present_flag() const noexcept;
  [[nodiscard]] auto aaps_log2_max_atlas_frame_order_cnt_lsb_minus4() const -> uint8_t;
  [[nodiscard]] constexpr auto aaps_extension_present_flag() const noexcept;
  [[nodiscard]] constexpr auto aaps_vpcc_extension_present_flag() const noexcept;
  [[nodiscard]] constexpr auto aaps_extension_7bits() const noexcept;
  [[nodiscard]] auto aaps_vpcc_extension() const -> const AapsVpccExtension &;
  [[nodiscard]] auto aapsExtensionData() const -> const std::vector<bool> &;

  constexpr auto aaps_atlas_adaptation_parameter_set_id(uint8_t value) -> auto &;
  constexpr auto aaps_log2_max_afoc_present_flag(bool value) noexcept -> auto &;
  auto aaps_log2_max_atlas_frame_order_cnt_lsb_minus4(uint8_t value) noexcept
      -> AtlasAdaptationParameterSetRBSP &;
  constexpr auto aaps_extension_present_flag(bool value) noexcept -> auto &;
  auto aaps_vpcc_extension_present_flag(bool value) noexcept -> AtlasAdaptationParameterSetRBSP &;
  auto aaps_extension_7bits(uint8_t value) noexcept -> AtlasAdaptationParameterSetRBSP &;
  auto aaps_vpcc_extension(const AapsVpccExtension &value) noexcept
      -> AtlasAdaptationParameterSetRBSP &;
  auto aapsExtensionData(std::vector<bool> value) noexcept -> AtlasAdaptationParameterSetRBSP &;

  friend auto operator<<(std::ostream &stream, const AtlasAdaptationParameterSetRBSP &x)
      -> std::ostream &;

  auto operator==(const AtlasAdaptationParameterSetRBSP & /*other*/) const -> bool;
  auto operator!=(const AtlasAdaptationParameterSetRBSP & /*other*/) const -> bool;

  static auto decodeFrom(std::istream &stream) -> AtlasAdaptationParameterSetRBSP;

  void encodeTo(std::ostream &stream) const;

private:
  uint8_t m_aaps_atlas_adaptation_parameter_set_id{};
  bool m_aaps_log2_max_afoc_present_flag{};
  std::optional<uint8_t> m_aaps_log2_max_atlas_frame_order_cnt_lsb_minus4{};
  bool m_aaps_extension_present_flag{};
  std::optional<bool> m_aaps_vpcc_extension_present_flag{};
  std::optional<uint8_t> m_aaps_extension_7bits{};
  std::optional<AapsVpccExtension> m_aaps_vpcc_extension{};
  std::optional<std::vector<bool>> m_aapsExtensionData{};
};

} // namespace TMIV::MivBitstream

#include "AtlasAdaptationParameterSetRBSP.hpp"

#endif
