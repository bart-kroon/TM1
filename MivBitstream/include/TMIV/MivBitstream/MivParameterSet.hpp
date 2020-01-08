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

#ifndef _TMIV_MIVBITSTREAM_MIVPARAMETERSET_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto MivSequenceParams::msp_profile_idc() const noexcept { return m_msp_profile_idc; }

constexpr auto MivSequenceParams::msp_depth_params_num_bits() const noexcept {
  return m_msp_depth_params_num_bits;
}

constexpr auto &MivSequenceParams::view_params_list() const noexcept { return m_view_params_list; }

constexpr auto MivSequenceParams::msp_depth_low_quality_flag() const noexcept {
  return m_msp_depth_low_quality_flag;
}

constexpr auto MivSequenceParams::msp_num_groups() const noexcept { return m_msp_num_groups; }

constexpr auto MivSequenceParams::msp_max_entities() const noexcept { return m_msp_max_entities; }

constexpr auto MivSequenceParams::msp_viewing_space_present_flag() const noexcept {
  return !!m_viewing_space;
}

constexpr auto MivSequenceParams::msp_extension_present_flag() const noexcept {
  return m_msp_extension_present_flag;
}

constexpr auto &MivSequenceParams::msp_profile_idc(MspProfileIdc value) noexcept {
  m_msp_profile_idc = value;
  return *this;
}

constexpr auto &MivSequenceParams::msp_depth_params_num_bits(std::uint8_t value) noexcept {
  m_msp_depth_params_num_bits = value;
  return *this;
}

constexpr auto &MivSequenceParams::msp_depth_low_quality_flag(bool value) noexcept {
  m_msp_depth_low_quality_flag = value;
  return *this;
}

constexpr auto &MivSequenceParams::msp_num_groups(std::size_t value) noexcept {
  m_msp_num_groups = value;
  return *this;
}

constexpr auto &MivSequenceParams::msp_max_entities(std::size_t value) noexcept {
  m_msp_max_entities = value;
  return *this;
}

constexpr auto &MivSequenceParams::msp_extension_present_flag(bool value) noexcept {
  m_msp_extension_present_flag = value;
  return *this;
}

constexpr auto MivParameterSet::miv_sequence_params_present_flag() const noexcept {
  return vps_extension_present_flag();
}
} // namespace TMIV::MivBitstream
