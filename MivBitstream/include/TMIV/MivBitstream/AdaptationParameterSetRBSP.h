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

#ifndef _TMIV_MIVBITSTREAM_ADAPTATIONPARAMETERSETRBSP_H_
#define _TMIV_MIVBITSTREAM_ADAPTATIONPARAMETERSETRBSP_H_

#include <TMIV/Common/Bitstream.h>

#include <iosfwd>
#include <optional>

namespace TMIV::MivBitstream {
// 23090-12: miv_view_params_list
class MivViewParamsList {
public:
  friend auto operator<<(std::ostream &stream, const MivViewParamsList & /* x */)
      -> std::ostream & {
    return stream;
  }

  constexpr auto operator==(const MivViewParamsList &) const noexcept { return true; }
  constexpr auto operator!=(const MivViewParamsList &) const noexcept { return false; }

  static auto decodeFrom(Common::InputBitstream &bitstream) -> MivViewParamsList { return {}; }

  void encodeTo(Common::OutputBitstream &bitstream) const {}
};

// 23090-12: miv_view_params_update_extrinsics
//
// TODO(BK): To be implemented in TMIV 4.1
class MivViewParamsUpdateExtrinsics {
public:
  friend auto operator<<(std::ostream &stream, const MivViewParamsUpdateExtrinsics & /* x */)
      -> std::ostream & {
    return stream;
  }

  constexpr auto operator==(const MivViewParamsUpdateExtrinsics &) const noexcept { return true; }
  constexpr auto operator!=(const MivViewParamsUpdateExtrinsics &) const noexcept { return false; }

  static auto decodeFrom(Common::InputBitstream &bitstream) -> MivViewParamsUpdateExtrinsics {
    return {};
  }

  void encodeTo(Common::OutputBitstream &bitstream) const {}
};

// 23090-12: miv_view_params_update_intrinsics
//
// TODO(BK): To be implemented in TMIV 4.1
class MivViewParamsUpdateIntrinsics {
public:
  friend auto operator<<(std::ostream &stream, const MivViewParamsUpdateIntrinsics & /* x */)
      -> std::ostream & {
    return stream;
  }

  constexpr auto operator==(const MivViewParamsUpdateIntrinsics &) const noexcept { return true; }
  constexpr auto operator!=(const MivViewParamsUpdateIntrinsics &) const noexcept { return false; }

  static auto decodeFrom(Common::InputBitstream &bitstream) -> MivViewParamsUpdateIntrinsics {
    return {};
  }

  void encodeTo(Common::OutputBitstream &bitstream) const {}
};

// 23090-12: ap_miv_view_params_list_update_mode
enum class MvplUpdateMode : std::uint8_t { VPL_INITLIST, VPL_UPD_EXT, VPL_UPD_INT, VPL_EXT_INT };
auto operator<<(std::ostream &stream, const MvplUpdateMode &x) -> std::ostream &;

// 23090-12: adapation_parameter_set_rbsp
class AdaptationParameterSetRBSP {
public:
  constexpr auto aps_adaptation_parameter_set_id() const noexcept;
  constexpr auto aps_camera_params_present_flag() const noexcept { return false; }
  constexpr auto aps_miv_view_params_list_present_flag() const noexcept;
  auto aps_miv_view_params_list_update_mode() const noexcept -> MvplUpdateMode;
  auto miv_view_params_list() const noexcept -> const MivViewParamsList &;
  auto miv_view_params_update_extrinsics() const noexcept -> const MivViewParamsUpdateExtrinsics &;
  auto miv_view_params_update_intrinsics() const noexcept -> const MivViewParamsUpdateIntrinsics &;
  constexpr auto aps_extension2_flag() const noexcept { return false; }

  constexpr auto &aps_adaptation_parameter_set_id(const std::uint8_t value) noexcept;
  constexpr auto &aps_miv_view_params_list_present_flag(const bool value) noexcept;
  constexpr auto aps_miv_view_params_list_update_mode(const MvplUpdateMode value) noexcept;
  [[nodiscard]] constexpr auto miv_view_params_list() noexcept -> MivViewParamsList &;
  [[nodiscard]] constexpr auto miv_view_params_update_extrinsics() noexcept
      -> MivViewParamsUpdateExtrinsics &;
  [[nodiscard]] constexpr auto miv_view_params_update_intrinsics() noexcept
      -> MivViewParamsUpdateIntrinsics &;

  friend auto operator<<(std::ostream &stream, const AdaptationParameterSetRBSP &x)
      -> std::ostream &;

  constexpr auto operator==(const AdaptationParameterSetRBSP &) const noexcept;
  constexpr auto operator!=(const AdaptationParameterSetRBSP &) const noexcept;

  static auto decodeFrom(std::istream &stream) -> AdaptationParameterSetRBSP;

  void encodeTo(std::ostream &stream) const;

private:
  std::uint8_t m_aps_adaptation_parameter_set_id{};
  bool m_aps_miv_view_params_list_present_flag{};
  std::optional<MvplUpdateMode> m_aps_miv_view_params_list_update_mode{};
  std::optional<MivViewParamsList> m_miv_view_params_list;
  std::optional<MivViewParamsUpdateExtrinsics> m_miv_view_params_update_extrinsics;
  std::optional<MivViewParamsUpdateIntrinsics> m_miv_view_params_update_intrinsics;
};
} // namespace TMIV::MivBitstream

#include "AdaptationParameterSetRBSP.hpp"

#endif
