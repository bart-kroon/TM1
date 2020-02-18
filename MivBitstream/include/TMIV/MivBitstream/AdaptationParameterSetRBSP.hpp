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

constexpr auto AdaptationParameterSetRBSP::aps_miv_view_params_list_update_mode(
    const MvplUpdateMode value) noexcept {
  m_aps_miv_view_params_list_update_mode = value;
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

constexpr auto AdaptationParameterSetRBSP::operator==(const AdaptationParameterSetRBSP &other) const
    noexcept {
  if (aps_adaptation_parameter_set_id() != other.aps_adaptation_parameter_set_id() ||
      aps_miv_view_params_list_present_flag() != other.aps_miv_view_params_list_present_flag()) {
    return false;
  }
  if (!aps_miv_view_params_list_present_flag()) {
    return true;
  }
  switch (aps_miv_view_params_list_update_mode()) {
  case MvplUpdateMode::VPL_INITLIST:
    return miv_view_params_list() == other.miv_view_params_list();
  case MvplUpdateMode::VPL_UPD_EXT:
    return miv_view_params_update_extrinsics() == other.miv_view_params_update_extrinsics();
  case MvplUpdateMode::VPL_UPD_INT:
    return miv_view_params_update_intrinsics() == other.miv_view_params_update_intrinsics();
  case MvplUpdateMode::VPL_EXT_INT:
    return miv_view_params_update_extrinsics() == other.miv_view_params_update_extrinsics() &&
           miv_view_params_update_intrinsics() == other.miv_view_params_update_intrinsics();
  default:
    return false;
  }
}

constexpr auto AdaptationParameterSetRBSP::operator!=(const AdaptationParameterSetRBSP &other) const
    noexcept {
  return !operator==(other);
}
} // namespace TMIV::MivBitstream
