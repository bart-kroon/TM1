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
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto DepthQuantization::dq_quantization_law() const noexcept { return uint8_t(0); }

constexpr auto DepthQuantization::dq_norm_disp_low() const noexcept { return m_dq_norm_disp_low; }

constexpr auto DepthQuantization::dq_norm_disp_high() const noexcept { return m_dq_norm_disp_high; }

constexpr auto DepthQuantization::dq_depth_occ_map_threshold_default() const noexcept {
  return m_dq_depth_occ_map_threshold_default;
}

constexpr auto &DepthQuantization::dq_norm_disp_low(const float value) noexcept {
  m_dq_norm_disp_low = value;
  return *this;
}

constexpr auto &DepthQuantization::dq_norm_disp_high(const float value) noexcept {
  m_dq_norm_disp_high = value;
  return *this;
}

constexpr auto &
DepthQuantization::dq_depth_occ_map_threshold_default(const std::uint32_t value) noexcept {
  m_dq_depth_occ_map_threshold_default = value;
  return *this;
}

constexpr auto DepthQuantization::operator==(const DepthQuantization &other) const noexcept {
  return dq_norm_disp_low() == other.dq_norm_disp_low() &&
         dq_norm_disp_high() == other.dq_norm_disp_high() &&
         dq_depth_occ_map_threshold_default() == other.dq_depth_occ_map_threshold_default();
}

constexpr auto DepthQuantization::operator!=(const DepthQuantization &other) const noexcept {
  return !operator==(other);
}

constexpr auto MivViewParamsList::mvp_intrinsic_params_equal_flag() const noexcept {
  return m_mvp_intrinsic_params_equal_flag;
}

constexpr auto MivViewParamsList::mvp_depth_quantization_params_equal_flag() const noexcept {
  return m_mvp_depth_quantization_params_equal_flag;
}

constexpr auto MivViewParamsList::mvp_pruning_graph_params_present_flag() const noexcept {
  return m_mvp_pruning_graph_params_present_flag;
}

constexpr auto AdaptationParameterSetRBSP::aps_adaptation_parameter_set_id() const noexcept {
  return m_aps_adaptation_parameter_set_id;
}

constexpr auto AdaptationParameterSetRBSP::aps_miv_view_params_list_present_flag() const noexcept {
  return m_aps_miv_view_params_list_present_flag;
}

constexpr auto &
AdaptationParameterSetRBSP::aps_adaptation_parameter_set_id(const std::uint8_t value) noexcept {
  m_aps_adaptation_parameter_set_id = value;
  return *this;
}

constexpr auto &
AdaptationParameterSetRBSP::aps_miv_view_params_list_present_flag(const bool value) noexcept {
  m_aps_miv_view_params_list_present_flag = value;
  return *this;
}

constexpr auto AdaptationParameterSetRBSP::miv_view_params_list() noexcept -> MivViewParamsList & {
  if (!m_miv_view_params_list) {
    m_miv_view_params_list = MivViewParamsList{};
  }
  return *m_miv_view_params_list;
}

constexpr auto AdaptationParameterSetRBSP::miv_view_params_update_extrinsics() noexcept
    -> MivViewParamsUpdateExtrinsics & {
  if (!m_miv_view_params_update_extrinsics) {
    m_miv_view_params_update_extrinsics = MivViewParamsUpdateExtrinsics{};
  }
  return *m_miv_view_params_update_extrinsics;
}

constexpr auto AdaptationParameterSetRBSP::miv_view_params_update_intrinsics() noexcept
    -> MivViewParamsUpdateIntrinsics & {
  if (!m_miv_view_params_update_intrinsics) {
    m_miv_view_params_update_intrinsics = MivViewParamsUpdateIntrinsics{};
  }
  return *m_miv_view_params_update_intrinsics;
}
} // namespace TMIV::MivBitstream
