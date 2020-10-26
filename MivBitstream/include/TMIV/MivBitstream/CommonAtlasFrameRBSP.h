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

#ifndef _TMIV_MIVBITSTREAM_COMMONATLASFRAMERBSP_H_
#define _TMIV_MIVBITSTREAM_COMMONATLASFRAMERBSP_H_

// TODO (CB) remove this temporary include (only the next line) when adapting to m55208
#include <TMIV/MivBitstream/CommonAtlasFrameMivExtension.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Matrix.h>
#include <TMIV/Common/Quaternion.h>
#include <TMIV/Common/Vector.h>

#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
// 23090-12: common_atlas_frame_rbsp( )
class CommonAtlasFrameRBSP {
public:
  [[nodiscard]] constexpr auto caf_atlas_adaptation_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto caf_frm_order_cnt_lsb() const noexcept;
  [[nodiscard]] constexpr auto caf_irap_flag() const noexcept;
  [[nodiscard]] auto caf_update_extrinsics_flag() const noexcept -> bool;
  [[nodiscard]] auto caf_update_intrinsics_flag() const noexcept -> bool;
  [[nodiscard]] auto caf_update_depth_quantization_flag() const noexcept -> bool;
  [[nodiscard]] auto miv_view_params_list() const noexcept -> const MivViewParamsList &;
  [[nodiscard]] auto miv_view_params_update_extrinsics() const noexcept
      -> const MivViewParamsUpdateExtrinsics &;
  [[nodiscard]] auto miv_view_params_update_intrinsics() const noexcept
      -> const MivViewParamsUpdateIntrinsics &;
  [[nodiscard]] auto miv_view_params_update_depth_quantization() const noexcept
      -> const MivViewParamsUpdateDepthQuantization &;
  [[nodiscard]] constexpr auto caf_extension_present_flag() const noexcept;
  [[nodiscard]] constexpr auto caf_extension_8bits() const noexcept;
  [[nodiscard]] auto cafExtensionData() const noexcept -> const std::vector<bool> &;

  constexpr auto caf_atlas_adaptation_parameter_set_id(std::uint8_t value) noexcept -> auto &;
  constexpr auto caf_frm_order_cnt_lsb(std::uint16_t value) noexcept -> auto &;
  constexpr auto caf_irap_flag(bool value) noexcept -> auto &;
  auto caf_update_extrinsics_flag(bool value) noexcept -> CommonAtlasFrameRBSP &;
  auto caf_update_intrinsics_flag(bool value) noexcept -> CommonAtlasFrameRBSP &;
  auto caf_update_depth_quantization_flag(bool value) noexcept -> CommonAtlasFrameRBSP &;
  [[nodiscard]] auto miv_view_params_list() noexcept -> MivViewParamsList &;
  [[nodiscard]] auto miv_view_params_update_extrinsics() noexcept
      -> MivViewParamsUpdateExtrinsics &;
  [[nodiscard]] auto miv_view_params_update_intrinsics() noexcept
      -> MivViewParamsUpdateIntrinsics &;
  [[nodiscard]] auto miv_view_params_update_depth_quantization() noexcept
      -> MivViewParamsUpdateDepthQuantization &;
  constexpr auto caf_extension_present_flag(bool value) noexcept -> auto &;
  auto caf_extension_8bits(std::uint8_t value) noexcept -> CommonAtlasFrameRBSP &;
  auto cafExtensionData(std::vector<bool> value) noexcept -> CommonAtlasFrameRBSP &;

  friend auto operator<<(std::ostream &stream, const CommonAtlasFrameRBSP &x) -> std::ostream &;

  auto operator==(const CommonAtlasFrameRBSP &) const noexcept -> bool;
  auto operator!=(const CommonAtlasFrameRBSP &) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream, const V3cParameterSet &vps,
                         unsigned maxCommonAtlasFrmOrderCntLsb) -> CommonAtlasFrameRBSP;

  void encodeTo(std::ostream &stream, const V3cParameterSet &vps,
                unsigned maxCommonAtlasFrmOrderCntLsb) const;

private:
  std::uint8_t m_caf_atlas_adaptation_parameter_set_id{};
  std::uint16_t m_caf_frm_order_cnt_lsb{};
  bool m_caf_irap_flag{true};
  bool m_caf_update_extrinsics_flag{};
  bool m_caf_update_intrinsics_flag{};
  bool m_caf_update_depth_quantization_flag{};
  std::optional<MivViewParamsList> m_miv_view_params_list;
  std::optional<MivViewParamsUpdateExtrinsics> m_miv_view_params_update_extrinsics;
  std::optional<MivViewParamsUpdateIntrinsics> m_miv_view_params_update_intrinsics;
  std::optional<MivViewParamsUpdateDepthQuantization> m_miv_view_params_update_depth_quantization;
  bool m_caf_extension_present_flag{};
  std::optional<std::uint8_t> m_caf_extension_8bits{};
  std::optional<std::vector<bool>> m_cafExtensionData{};
};
} // namespace TMIV::MivBitstream

#include "CommonAtlasFrameRBSP.hpp"

#endif
