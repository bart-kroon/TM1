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

#include <TMIV/MivBitstream/CommonAtlasFrameRBSP.h>

#include <TMIV/Common/verify.h>

#include <cmath>
#include <fmt/ostream.h>

namespace TMIV::MivBitstream {
auto CommonAtlasFrameRBSP::caf_miv_extension() const -> const CafMivExtension & {
  VERIFY_V3CBITSTREAM(caf_miv_extension_present_flag());
  VERIFY_V3CBITSTREAM(m_caf_miv_extension.has_value());
  return *m_caf_miv_extension;
}

auto CommonAtlasFrameRBSP::cafExtensionData() const -> const std::vector<bool> & {
  VERIFY_V3CBITSTREAM(caf_extension_7bits() != 0);
  VERIFY_V3CBITSTREAM(m_cafExtensionData.has_value());
  return *m_cafExtensionData;
}

auto CommonAtlasFrameRBSP::caf_miv_extension() noexcept -> CafMivExtension & {
  caf_miv_extension_present_flag(true);
  if (!m_caf_miv_extension.has_value()) {
    m_caf_miv_extension = CafMivExtension{};
  }
  return *m_caf_miv_extension;
}

auto CommonAtlasFrameRBSP::cafExtensionData(std::vector<bool> value) noexcept
    -> CommonAtlasFrameRBSP & {
  PRECONDITION(caf_extension_7bits() != 0);
  m_cafExtensionData = std::move(value);
  return *this;
}

auto CommonAtlasFrameRBSP::printTo(
    std::ostream &stream, const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV) const
    -> std::ostream & {
  const auto &casps = caspsById(caspsV, caf_common_atlas_sequence_parameter_set_id());
  fmt::print(stream, "caf_common_atlas_sequence_parameter_set_id={}\n",
             int32_t{caf_common_atlas_sequence_parameter_set_id()});
  fmt::print(stream, "caf_common_atlas_frm_order_cnt_lsb={}\n",
             caf_common_atlas_frm_order_cnt_lsb());
  fmt::print(stream, "caf_extension_present_flag={}\n", caf_extension_present_flag());
  if (caf_extension_present_flag()) {
    fmt::print(stream, "caf_miv_extension_present_flag={}\n", caf_miv_extension_present_flag());
    fmt::print(stream, "caf_extension_7bits={}\n", int32_t{caf_extension_7bits()});
  }
  if (caf_miv_extension_present_flag()) {
    caf_miv_extension().printTo(stream, casps);
  }
  if (caf_extension_7bits() != 0U) {
    for (const auto bit : cafExtensionData()) {
      fmt::print(stream, "caf_extension_data_flag={}\n", bit);
    }
  }
  return stream;
}

auto CommonAtlasFrameRBSP::operator==(const CommonAtlasFrameRBSP &other) const -> bool {
  if (caf_common_atlas_sequence_parameter_set_id() !=
          other.caf_common_atlas_sequence_parameter_set_id() ||
      caf_common_atlas_frm_order_cnt_lsb() != other.caf_common_atlas_frm_order_cnt_lsb() ||
      caf_extension_present_flag() != other.caf_extension_present_flag() ||
      caf_extension_7bits() != other.caf_extension_7bits()) {
    return false;
  }
  return caf_extension_7bits() == 0 || cafExtensionData() == other.cafExtensionData();
}

auto CommonAtlasFrameRBSP::operator!=(const CommonAtlasFrameRBSP &other) const -> bool {
  return !operator==(other);
}

auto CommonAtlasFrameRBSP::decodeFrom(
    std::istream &stream, const NalUnitHeader &nuh,
    const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV,
    uint32_t maxCommonAtlasFrmOrderCntLsb) -> CommonAtlasFrameRBSP {
  Common::InputBitstream bitstream{stream};

  auto x = CommonAtlasFrameRBSP{};

  x.caf_common_atlas_sequence_parameter_set_id(bitstream.readBits<uint8_t>(4));
  const auto &casps = caspsById(caspsV, x.caf_common_atlas_sequence_parameter_set_id());

  x.caf_common_atlas_frm_order_cnt_lsb(bitstream.getUVar<uint16_t>(maxCommonAtlasFrmOrderCntLsb));
  x.caf_extension_present_flag(bitstream.getFlag());

  if (x.caf_extension_present_flag()) {
    x.caf_miv_extension_present_flag(bitstream.getFlag());
    x.caf_extension_7bits(bitstream.readBits<uint8_t>(7));
  }
  if (x.caf_miv_extension_present_flag()) {
    x.caf_miv_extension() = CafMivExtension::decodeFrom(bitstream, nuh, casps);
  }
  if (x.caf_extension_7bits() != 0) {
    auto cafExtensionData = std::vector<bool>{};
    while (bitstream.moreRbspData()) {
      cafExtensionData.push_back(bitstream.getFlag());
    }
    x.cafExtensionData(std::move(cafExtensionData));
  }
  bitstream.rbspTrailingBits();

  return x;
}

void CommonAtlasFrameRBSP::encodeTo(std::ostream &stream, const NalUnitHeader &nuh,
                                    const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV,
                                    uint32_t maxCommonAtlasFrmOrderCntLsb) const {
  PRECONDITION(0 < maxCommonAtlasFrmOrderCntLsb);
  Common::OutputBitstream bitstream{stream};

  bitstream.writeBits(caf_common_atlas_sequence_parameter_set_id(), 4);
  const auto &casps = caspsById(caspsV, caf_common_atlas_sequence_parameter_set_id());

  bitstream.putUVar(caf_common_atlas_frm_order_cnt_lsb(), maxCommonAtlasFrmOrderCntLsb);
  bitstream.putFlag(caf_extension_present_flag());

  if (caf_extension_present_flag()) {
    bitstream.putFlag(caf_miv_extension_present_flag());
    bitstream.writeBits(caf_extension_7bits(), 7);
  }
  if (caf_miv_extension_present_flag()) {
    caf_miv_extension().encodeTo(bitstream, nuh, casps);
  }
  if (caf_extension_7bits() != 0) {
    for (auto bit : cafExtensionData()) {
      bitstream.putFlag(bit);
    }
  }
  bitstream.rbspTrailingBits();
}

auto DepthQuantization::dq_pivot_norm_disp(int32_t index) const noexcept -> float {
  return m_dq_pivot_norm_disp[index];
}

auto DepthQuantization::dq_pivot_norm_disp(int32_t index, float value) noexcept
    -> DepthQuantization & {
  if (static_cast<uint8_t>(m_dq_pivot_norm_disp.size()) < index + 1) {
    m_dq_pivot_norm_disp.resize(index + 1);
  }
  m_dq_pivot_norm_disp[index] = value;
  return *this;
}
} // namespace TMIV::MivBitstream
