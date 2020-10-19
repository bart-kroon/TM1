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

#include <TMIV/MivBitstream/CommonAtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/verify.h>

namespace TMIV::MivBitstream {
auto CaspsMivExtension::vui_parameters() const noexcept -> const VuiParameters & {
  VERIFY_MIVBITSTREAM(casme_vui_params_present_flag());
  VERIFY_MIVBITSTREAM(m_vui_parameters.has_value());
  return *m_vui_parameters;
}

auto CaspsMivExtension::vui_parameters(const VuiParameters &value) noexcept -> CaspsMivExtension & {
  VERIFY_MIVBITSTREAM(casme_vui_params_present_flag());
  m_vui_parameters = value;
  return *this;
}

auto operator<<(std::ostream &stream, const CaspsMivExtension &x) -> std::ostream & {
  stream << "casme_omaf_v1_compatible_flag=" << std::boolalpha << x.casme_omaf_v1_compatible_flag()
         << '\n';
  stream << "casme_vui_params_present_flag=" << std::boolalpha << x.casme_vui_params_present_flag()
         << '\n';
  if (x.casme_vui_params_present_flag()) {
    stream << x.vui_parameters();
  }
  return stream;
}

auto CaspsMivExtension::operator==(const CaspsMivExtension &other) const noexcept -> bool {
  return casme_omaf_v1_compatible_flag() == other.casme_omaf_v1_compatible_flag();
}

auto CaspsMivExtension::operator!=(const CaspsMivExtension &other) const noexcept -> bool {
  return !operator==(other);
}

auto CaspsMivExtension::decodeFrom(Common::InputBitstream &bitstream) -> CaspsMivExtension {
  auto x = CaspsMivExtension{};
  x.casme_omaf_v1_compatible_flag(bitstream.getFlag());
  x.casme_vui_params_present_flag(bitstream.getFlag());
  if (x.casme_vui_params_present_flag()) {
    x.vui_parameters(VuiParameters::decodeFrom(bitstream, nullptr));
  }
  return x;
}

void CaspsMivExtension::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFlag(casme_omaf_v1_compatible_flag());
  bitstream.putFlag(casme_vui_params_present_flag());
  if (casme_vui_params_present_flag()) {
    vui_parameters().encodeTo(bitstream, nullptr);
  }
}

// TODO (CB) extract this method to common
template <typename T>
auto putField(std::ostream &stream, const std::string &fieldName, T &&fieldValue) {
  stream << fieldName << "=";
  if constexpr (std::is_same_v<std::uint8_t, std::decay_t<T>>) {
    stream << static_cast<unsigned>(fieldValue) << "\n";
  } else if (std::is_same_v<bool, std::decay_t<T>>) {
    stream << std::boolalpha << fieldValue << "\n";
  } else {
    stream << fieldValue << "\n";
  }
}

auto operator<<(std::ostream &stream, const CommonAtlasSequenceParameterSetRBSP &x)
    -> std::ostream & {
  putField(stream, "casps_common_atlas_sequence_parameter_set_id",
           x.casps_common_atlas_sequence_parameter_set_id());
  putField(stream, "casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4",
           x.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4());
  putField(stream, "casps_extension_present_flag", x.casps_extension_present_flag());
  return stream;
}

auto CommonAtlasSequenceParameterSetRBSP::operator==(
    const CommonAtlasSequenceParameterSetRBSP &other) const noexcept -> bool {
  return (m_casps_common_atlas_sequence_parameter_set_id ==
          other.m_casps_common_atlas_sequence_parameter_set_id) &&
         (m_casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4 ==
          other.m_casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4) &&
         (m_casps_extension_present_flag == other.m_casps_extension_present_flag);
}

auto CommonAtlasSequenceParameterSetRBSP::operator!=(
    const CommonAtlasSequenceParameterSetRBSP &other) const noexcept -> bool {
  return !operator==(other);
}

auto CommonAtlasSequenceParameterSetRBSP::decodeFrom(Common::InputBitstream &bitstream)
    -> CommonAtlasSequenceParameterSetRBSP {
  CommonAtlasSequenceParameterSetRBSP result{};
  result.casps_common_atlas_sequence_parameter_set_id(bitstream.readBits<std::uint8_t>(4));
  result.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4(
      bitstream.getUExpGolomb<std::size_t>());
  result.casps_extension_present_flag(bitstream.getFlag());
  return result;
}

void CommonAtlasSequenceParameterSetRBSP::encodeTo(Common::OutputBitstream &stream) const {
  stream.writeBits(casps_common_atlas_sequence_parameter_set_id(), 4);
  stream.putUExpGolomb(casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4());
  stream.putFlag(casps_extension_present_flag());
}

} // namespace TMIV::MivBitstream