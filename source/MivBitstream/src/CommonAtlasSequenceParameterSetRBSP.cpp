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

#include <TMIV/MivBitstream/CommonAtlasSequenceParameterSetRBSP.h>

#include <TMIV/Common/verify.h>

#include <fmt/ostream.h>

#include <algorithm>

namespace TMIV::MivBitstream {
auto CaspsMivExtension::vui_parameters() const -> const VuiParameters & {
  VERIFY_MIVBITSTREAM(casme_vui_params_present_flag());
  VERIFY_MIVBITSTREAM(m_vui_parameters.has_value());
  return *m_vui_parameters;
}

auto CaspsMivExtension::vui_parameters(const VuiParameters &value) noexcept -> CaspsMivExtension & {
  casme_vui_params_present_flag(true);
  m_vui_parameters = value;
  return *this;
}

auto operator<<(std::ostream &stream, const CaspsMivExtension &x) -> std::ostream & {
  fmt::print(stream, "casme_depth_low_quality_flag={}\n", x.casme_depth_low_quality_flag());
  fmt::print(stream, "casme_depth_quantization_params_present_flag={}\n",
             x.casme_depth_quantization_params_present_flag());
  fmt::print(stream, "casme_vui_params_present_flag={}\n", x.casme_vui_params_present_flag());

  if (x.casme_vui_params_present_flag()) {
    stream << x.vui_parameters();
  }
  return stream;
}

auto CaspsMivExtension::operator==(const CaspsMivExtension &other) const noexcept -> bool {
  return casme_depth_low_quality_flag() == other.casme_depth_low_quality_flag() &&
         casme_depth_quantization_params_present_flag() ==
             other.casme_depth_quantization_params_present_flag() &&
         casme_vui_params_present_flag() == other.casme_vui_params_present_flag();
}

auto CaspsMivExtension::operator!=(const CaspsMivExtension &other) const noexcept -> bool {
  return !operator==(other);
}

auto CaspsMivExtension::decodeFrom(Common::InputBitstream &bitstream) -> CaspsMivExtension {
  auto x = CaspsMivExtension{};
  x.casme_depth_low_quality_flag(bitstream.getFlag());
  x.casme_depth_quantization_params_present_flag(bitstream.getFlag());
  x.casme_vui_params_present_flag(bitstream.getFlag());

  if (x.casme_vui_params_present_flag()) {
    x.vui_parameters(VuiParameters::decodeFrom(bitstream, nullptr));
  }
  return x;
}

void CaspsMivExtension::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFlag(casme_depth_low_quality_flag());
  bitstream.putFlag(casme_depth_quantization_params_present_flag());
  bitstream.putFlag(casme_vui_params_present_flag());
  if (casme_vui_params_present_flag()) {
    vui_parameters().encodeTo(bitstream, nullptr);
  }
}

auto operator<<(std::ostream &stream, const CaspsMiv2Extension &x) -> std::ostream & {
  fmt::print(stream, "casme_decoder_side_depth_estimation_flag={}\n",
             x.casme_decoder_side_depth_estimation_flag());
  fmt::print(stream, "casme_chroma_scaling_present_flag={}\n",
             x.casme_chroma_scaling_present_flag());
  fmt::print(stream, "casme_capture_device_information_present_flag=false\n");

  return stream;
}

auto CaspsMiv2Extension::operator==(const CaspsMiv2Extension &other) const noexcept -> bool {
  return casme_decoder_side_depth_estimation_flag() ==
             other.casme_decoder_side_depth_estimation_flag() &&
         casme_chroma_scaling_present_flag() == other.casme_chroma_scaling_present_flag();
}

auto CaspsMiv2Extension::operator!=(const CaspsMiv2Extension &other) const noexcept -> bool {
  return !operator==(other);
}

auto CaspsMiv2Extension::decodeFrom(Common::InputBitstream &bitstream) -> CaspsMiv2Extension {
  auto x = CaspsMiv2Extension{};

  x.casme_decoder_side_depth_estimation_flag(bitstream.getFlag());
  x.casme_chroma_scaling_present_flag(bitstream.getFlag());

  const auto casme_capture_device_information_present_flag = bitstream.getFlag();

  if (casme_capture_device_information_present_flag) {
    NOT_IMPLEMENTED;
  }

  const auto casme_reserved_zero_8bits = bitstream.getUint8();
  VERIFY_MIVBITSTREAM(casme_reserved_zero_8bits == 0);

  return x;
}

void CaspsMiv2Extension::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFlag(casme_decoder_side_depth_estimation_flag());
  bitstream.putFlag(casme_chroma_scaling_present_flag());

  static constexpr auto casme_capture_device_information_present_flag = false;
  bitstream.putFlag(casme_capture_device_information_present_flag);

  static constexpr auto casme_reserved_zero_8bits = 0;
  bitstream.putUint8(casme_reserved_zero_8bits);
}

auto CommonAtlasSequenceParameterSetRBSP::casps_miv_extension_present_flag() const -> bool {
  return m_casps_miv_extension_present_flag.value_or(false);
}

auto CommonAtlasSequenceParameterSetRBSP::casps_miv_2_extension_present_flag() const -> bool {
  return m_casps_miv_2_extension_present_flag.value_or(false);
}

auto CommonAtlasSequenceParameterSetRBSP::casps_extension_6bits() const {
  return m_casps_extension_6bits.value_or(uint8_t{});
}

auto CommonAtlasSequenceParameterSetRBSP::casps_miv_extension() const -> const CaspsMivExtension & {
  VERIFY_V3CBITSTREAM(casps_miv_extension_present_flag());
  VERIFY_V3CBITSTREAM(m_casps_miv_extension.has_value());
  return *m_casps_miv_extension;
}

auto CommonAtlasSequenceParameterSetRBSP::casps_miv_2_extension() const
    -> const CaspsMiv2Extension & {
  VERIFY_V3CBITSTREAM(casps_miv_2_extension_present_flag());
  VERIFY_V3CBITSTREAM(m_casps_miv_2_extension.has_value());
  return *m_casps_miv_2_extension;
}

auto CommonAtlasSequenceParameterSetRBSP::caspsExtensionData() const -> const std::vector<bool> & {
  VERIFY_V3CBITSTREAM(casps_extension_6bits() != 0);
  VERIFY_V3CBITSTREAM(m_caspsExtensionData.has_value());
  return *m_caspsExtensionData;
}

auto CommonAtlasSequenceParameterSetRBSP::casps_miv_extension_present_flag(bool flag) noexcept
    -> CommonAtlasSequenceParameterSetRBSP & {
  casps_extension_present_flag(true);
  m_casps_miv_extension_present_flag = flag;
  return *this;
}

auto CommonAtlasSequenceParameterSetRBSP::casps_miv_2_extension_present_flag(bool flag) noexcept
    -> CommonAtlasSequenceParameterSetRBSP & {
  casps_extension_present_flag(true);
  m_casps_miv_2_extension_present_flag = flag;
  return *this;
}

auto CommonAtlasSequenceParameterSetRBSP::casps_extension_6bits(uint8_t value) noexcept
    -> CommonAtlasSequenceParameterSetRBSP & {
  casps_extension_present_flag(true);
  PRECONDITION(value < 0b100'0000);
  m_casps_extension_6bits = value;
  return *this;
}

auto CommonAtlasSequenceParameterSetRBSP::casps_miv_extension() noexcept -> CaspsMivExtension & {
  casps_miv_extension_present_flag(true);

  if (!m_casps_miv_extension.has_value()) {
    m_casps_miv_extension = CaspsMivExtension{};
  }
  return *m_casps_miv_extension;
}

auto CommonAtlasSequenceParameterSetRBSP::casps_miv_2_extension() noexcept -> CaspsMiv2Extension & {
  casps_miv_2_extension_present_flag(true);

  if (!m_casps_miv_2_extension.has_value()) {
    m_casps_miv_2_extension = CaspsMiv2Extension{};
  }
  return *m_casps_miv_2_extension;
}

auto CommonAtlasSequenceParameterSetRBSP::caspsExtensionData(std::vector<bool> value)
    -> CommonAtlasSequenceParameterSetRBSP & {
  PRECONDITION(casps_extension_6bits() != 0);
  m_caspsExtensionData = std::move(value);
  return *this;
}

auto operator<<(std::ostream &stream, const CommonAtlasSequenceParameterSetRBSP &x)
    -> std::ostream & {
  fmt::print(stream, "casps_common_atlas_sequence_parameter_set_id={}\n",
             x.casps_common_atlas_sequence_parameter_set_id());
  fmt::print(stream, "casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4={}\n",
             x.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4());
  fmt::print(stream, "casps_extension_present_flag={}\n", x.casps_extension_present_flag());

  if (x.casps_extension_present_flag()) {
    fmt::print(stream, "casps_miv_extension_present_flag={}\n",
               x.casps_miv_extension_present_flag());
    fmt::print(stream, "casps_miv_2_extension_present_flag={}\n",
               x.casps_miv_2_extension_present_flag());
    fmt::print(stream, "casps_extension_6bits={}\n", x.casps_extension_6bits());
  }
  if (x.casps_miv_extension_present_flag()) {
    stream << x.casps_miv_extension();
  }
  if (x.casps_miv_2_extension_present_flag()) {
    stream << x.casps_miv_2_extension();
  }
  if (x.casps_extension_6bits() != 0) {
    for (auto bit : x.caspsExtensionData()) {
      fmt::print(stream, "casps_extension_data_flag={}\n", bit);
    }
  }
  return stream;
}

auto CommonAtlasSequenceParameterSetRBSP::operator==(
    const CommonAtlasSequenceParameterSetRBSP &other) const noexcept -> bool {
  if (casps_common_atlas_sequence_parameter_set_id() !=
          other.casps_common_atlas_sequence_parameter_set_id() ||
      casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4() !=
          other.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4() ||
      casps_extension_present_flag() != other.casps_extension_present_flag()) {
    return false;
  }
  if (casps_extension_present_flag()) {
    if (casps_miv_extension_present_flag() != other.casps_miv_extension_present_flag() ||
        casps_miv_2_extension_present_flag() != other.casps_miv_2_extension_present_flag() ||
        casps_extension_6bits() != other.casps_extension_6bits()) {
      return false;
    }
  }
  if (casps_miv_extension_present_flag()) {
    if (casps_miv_extension() != other.casps_miv_extension()) {
      return false;
    }
  }
  if (casps_miv_2_extension_present_flag()) {
    if (casps_miv_2_extension() != other.casps_miv_2_extension()) {
      return false;
    }
  }
  if (casps_extension_6bits() != 0) {
    if (caspsExtensionData() != other.caspsExtensionData()) {
      return false;
    }
  }
  return true;
}

auto CommonAtlasSequenceParameterSetRBSP::operator!=(
    const CommonAtlasSequenceParameterSetRBSP &other) const noexcept -> bool {
  return !operator==(other);
}

auto CommonAtlasSequenceParameterSetRBSP::decodeFrom(std::istream &stream)
    -> CommonAtlasSequenceParameterSetRBSP {
  Common::InputBitstream bitstream{stream};
  CommonAtlasSequenceParameterSetRBSP result{};

  result.casps_common_atlas_sequence_parameter_set_id(bitstream.readBits<uint8_t>(4));
  result.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4(bitstream.getUExpGolomb<uint8_t>());
  result.casps_extension_present_flag(bitstream.getFlag());
  if (result.casps_extension_present_flag()) {
    result.casps_miv_extension_present_flag(bitstream.getFlag());
    result.casps_miv_2_extension_present_flag(bitstream.getFlag());
    result.casps_extension_6bits(bitstream.readBits<uint8_t>(6));
  }
  if (result.casps_miv_extension_present_flag()) {
    result.casps_miv_extension() = CaspsMivExtension::decodeFrom(bitstream);
  }
  if (result.casps_miv_2_extension_present_flag()) {
    result.casps_miv_2_extension() = CaspsMiv2Extension::decodeFrom(bitstream);
  }
  if (result.casps_extension_6bits() != 0) {
    auto caspsExtensionData = std::vector<bool>{};
    while (bitstream.moreRbspData()) {
      caspsExtensionData.push_back(bitstream.getFlag());
    }
    result.caspsExtensionData(std::move(caspsExtensionData));
  }
  bitstream.rbspTrailingBits();

  return result;
}

void CommonAtlasSequenceParameterSetRBSP::encodeTo(std::ostream &stream) const {
  Common::OutputBitstream bitstream{stream};
  bitstream.writeBits(casps_common_atlas_sequence_parameter_set_id(), 4);
  bitstream.putUExpGolomb(casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4());
  bitstream.putFlag(casps_extension_present_flag());
  if (casps_extension_present_flag()) {
    bitstream.putFlag(casps_miv_extension_present_flag());
    bitstream.putFlag(casps_miv_2_extension_present_flag());
    bitstream.writeBits(casps_extension_6bits(), 6);
  }
  if (casps_miv_extension_present_flag()) {
    casps_miv_extension().encodeTo(bitstream);
  }
  if (casps_miv_2_extension_present_flag()) {
    casps_miv_2_extension().encodeTo(bitstream);
  }
  if (casps_extension_6bits() != 0) {
    for (const auto bit : caspsExtensionData()) {
      bitstream.putFlag(bit);
    }
  }
  bitstream.rbspTrailingBits();
}

auto caspsById(const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV, int32_t id)
    -> const CommonAtlasSequenceParameterSetRBSP & {
  const auto result = std::find_if(std::cbegin(caspsV), std::cend(caspsV), [id](const auto &casps) {
    return id == casps.casps_common_atlas_sequence_parameter_set_id();
  });

  if (result == std::cend(caspsV)) {
    V3CBITSTREAM_ERROR("Unknown CASPS ID");
  }

  return *result;
}

} // namespace TMIV::MivBitstream