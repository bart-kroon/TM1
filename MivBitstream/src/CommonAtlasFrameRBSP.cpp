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

#include <TMIV/MivBitstream/CommonAtlasFrameRBSP.h>

#include <TMIV/MivBitstream/verify.h>

#include <cmath>

namespace TMIV::MivBitstream {
auto CommonAtlasFrameRBSP::caf_update_extrinsics_flag() const noexcept -> bool {
  VERIFY_MIVBITSTREAM(!caf_irap_flag());
  return m_caf_update_extrinsics_flag;
}

auto CommonAtlasFrameRBSP::caf_update_intrinsics_flag() const noexcept -> bool {
  VERIFY_MIVBITSTREAM(!caf_irap_flag());
  return m_caf_update_intrinsics_flag;
}

auto CommonAtlasFrameRBSP::caf_update_depth_quantization_flag() const noexcept -> bool {
  VERIFY_MIVBITSTREAM(!caf_irap_flag());
  return m_caf_update_depth_quantization_flag;
}

auto CommonAtlasFrameRBSP::miv_view_params_list() const noexcept -> const MivViewParamsList & {
  VERIFY_MIVBITSTREAM(caf_irap_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_list.has_value());
  return *m_miv_view_params_list;
}

auto CommonAtlasFrameRBSP::miv_view_params_update_extrinsics() const noexcept
    -> const MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(caf_update_extrinsics_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_extrinsics.has_value());
  return *m_miv_view_params_update_extrinsics;
}

auto CommonAtlasFrameRBSP::miv_view_params_update_intrinsics() const noexcept
    -> const MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(caf_update_intrinsics_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_intrinsics.has_value());
  return *m_miv_view_params_update_intrinsics;
}

auto CommonAtlasFrameRBSP::miv_view_params_update_depth_quantization() const noexcept
    -> const MivViewParamsUpdateDepthQuantization & {
  VERIFY_MIVBITSTREAM(caf_update_depth_quantization_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_depth_quantization.has_value());
  return *m_miv_view_params_update_depth_quantization;
}

auto CommonAtlasFrameRBSP::miv_view_params_list() noexcept -> MivViewParamsList & {
  VERIFY_MIVBITSTREAM(caf_irap_flag());
  if (!m_miv_view_params_list) {
    m_miv_view_params_list = MivViewParamsList{};
  }
  return *m_miv_view_params_list;
}

auto CommonAtlasFrameRBSP::miv_view_params_update_extrinsics() noexcept
    -> MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(caf_update_extrinsics_flag());
  if (!m_miv_view_params_update_extrinsics) {
    m_miv_view_params_update_extrinsics = MivViewParamsUpdateExtrinsics{};
  }
  return *m_miv_view_params_update_extrinsics;
}

auto CommonAtlasFrameRBSP::miv_view_params_update_intrinsics() noexcept
    -> MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(caf_update_intrinsics_flag());
  if (!m_miv_view_params_update_intrinsics) {
    m_miv_view_params_update_intrinsics = MivViewParamsUpdateIntrinsics{};
  }
  return *m_miv_view_params_update_intrinsics;
}

auto CommonAtlasFrameRBSP::miv_view_params_update_depth_quantization() noexcept
    -> MivViewParamsUpdateDepthQuantization & {
  VERIFY_MIVBITSTREAM(caf_update_depth_quantization_flag());
  if (!m_miv_view_params_update_depth_quantization) {
    m_miv_view_params_update_depth_quantization = MivViewParamsUpdateDepthQuantization{};
  }
  return *m_miv_view_params_update_depth_quantization;
}

auto CommonAtlasFrameRBSP::cafExtensionData() const noexcept -> const std::vector<bool> & {
  VERIFY_V3CBITSTREAM(caf_extension_8bits() != 0);
  VERIFY_V3CBITSTREAM(m_cafExtensionData.has_value());
  return *m_cafExtensionData;
}

auto CommonAtlasFrameRBSP::caf_update_extrinsics_flag(bool value) noexcept
    -> CommonAtlasFrameRBSP & {
  VERIFY_MIVBITSTREAM(!caf_irap_flag());
  m_caf_update_extrinsics_flag = value;
  return *this;
}

auto CommonAtlasFrameRBSP::caf_update_intrinsics_flag(bool value) noexcept
    -> CommonAtlasFrameRBSP & {
  VERIFY_MIVBITSTREAM(!caf_irap_flag());
  m_caf_update_intrinsics_flag = value;
  return *this;
}

auto CommonAtlasFrameRBSP::caf_update_depth_quantization_flag(bool value) noexcept
    -> CommonAtlasFrameRBSP & {
  VERIFY_MIVBITSTREAM(!caf_irap_flag());
  m_caf_update_depth_quantization_flag = value;
  return *this;
}

auto CommonAtlasFrameRBSP::caf_extension_8bits(std::uint8_t value) noexcept
    -> CommonAtlasFrameRBSP & {
  VERIFY_V3CBITSTREAM(caf_extension_present_flag());
  m_caf_extension_8bits = value;
  return *this;
}

auto CommonAtlasFrameRBSP::cafExtensionData(std::vector<bool> value) noexcept
    -> CommonAtlasFrameRBSP & {
  VERIFY_V3CBITSTREAM(caf_extension_8bits() != 0);
  m_cafExtensionData = std::move(value);
  return *this;
}

auto operator<<(std::ostream &stream, const CommonAtlasFrameRBSP &x) -> std::ostream & {
  stream << "caf_atlas_adaptation_parameter_set_id="
         << int{x.caf_atlas_adaptation_parameter_set_id()} << '\n';
  stream << "caf_frm_order_cnt_lsb=" << x.caf_frm_order_cnt_lsb() << '\n';
  stream << "caf_irap_flag=" << std::boolalpha << x.caf_irap_flag() << '\n';
  if (x.caf_irap_flag()) {
    stream << x.miv_view_params_list();
  } else {
    stream << "caf_update_extrinsics_flag=" << std::boolalpha << x.caf_update_extrinsics_flag()
           << '\n';
    stream << "caf_update_intrinsics_flag=" << std::boolalpha << x.caf_update_intrinsics_flag()
           << '\n';
    stream << "caf_update_depth_quantization_flag=" << std::boolalpha
           << x.caf_update_depth_quantization_flag() << '\n';
    if (x.caf_update_extrinsics_flag()) {
      stream << x.miv_view_params_update_extrinsics();
    }
    if (x.caf_update_intrinsics_flag()) {
      stream << x.miv_view_params_update_intrinsics();
    }
    if (x.caf_update_depth_quantization_flag()) {
      stream << x.miv_view_params_update_depth_quantization();
    }
  }

  stream << "caf_extension_present_flag=" << std::boolalpha << x.caf_extension_present_flag()
         << '\n';
  if (x.caf_extension_present_flag()) {
    stream << "caf_extension_8bits=" << int{x.caf_extension_8bits()} << '\n';
  }
  if (x.caf_extension_8bits() != 0U) {
    for (auto bit : x.cafExtensionData()) {
      stream << "caf_extension_data_flag=" << std::boolalpha << bit << '\n';
    }
  }
  return stream;
}

auto CommonAtlasFrameRBSP::operator==(const CommonAtlasFrameRBSP &other) const noexcept -> bool {
  if (caf_atlas_adaptation_parameter_set_id() != other.caf_atlas_adaptation_parameter_set_id() ||
      caf_frm_order_cnt_lsb() != other.caf_frm_order_cnt_lsb() ||
      caf_irap_flag() != other.caf_irap_flag() ||
      caf_extension_present_flag() != other.caf_extension_present_flag() ||
      caf_extension_8bits() != other.caf_extension_8bits()) {
    return false;
  }
  if (caf_irap_flag()) {
    if (miv_view_params_list() != other.miv_view_params_list()) {
      return false;
    }
  } else {
    if (caf_update_extrinsics_flag() != other.caf_update_extrinsics_flag() ||
        caf_update_intrinsics_flag() != other.caf_update_intrinsics_flag() ||
        caf_update_depth_quantization_flag() != other.caf_update_depth_quantization_flag()) {
      return false;
    }
    if (caf_update_extrinsics_flag() &&
        miv_view_params_update_extrinsics() != other.miv_view_params_update_extrinsics()) {
      return false;
    }
    if (caf_update_intrinsics_flag() &&
        miv_view_params_update_intrinsics() != other.miv_view_params_update_intrinsics()) {
      return false;
    }
    if (caf_update_depth_quantization_flag() &&
        miv_view_params_update_depth_quantization() !=
            other.miv_view_params_update_depth_quantization()) {
      return false;
    }
  }

  return caf_extension_8bits() == 0 || cafExtensionData() == other.cafExtensionData();
}

auto CommonAtlasFrameRBSP::operator!=(const CommonAtlasFrameRBSP &other) const noexcept -> bool {
  return !operator==(other);
}

auto CommonAtlasFrameRBSP::decodeFrom(std::istream &stream, const V3cParameterSet &vps,
                                      unsigned maxCommonAtlasFrmOrderCntLsb)
    -> CommonAtlasFrameRBSP {
  Common::InputBitstream bitstream{stream};

  auto x = CommonAtlasFrameRBSP{};

  x.caf_atlas_adaptation_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());
  x.caf_frm_order_cnt_lsb(bitstream.getUVar<uint16_t>(maxCommonAtlasFrmOrderCntLsb));
  x.caf_irap_flag(bitstream.getFlag());

  if (x.caf_irap_flag()) {
    x.miv_view_params_list() = MivViewParamsList::decodeFrom(bitstream, vps);
  } else {
    x.caf_update_extrinsics_flag(bitstream.getFlag());
    x.caf_update_intrinsics_flag(bitstream.getFlag());
    x.caf_update_depth_quantization_flag(bitstream.getFlag());

    if (x.caf_update_extrinsics_flag()) {
      x.miv_view_params_update_extrinsics() = MivViewParamsUpdateExtrinsics::decodeFrom(bitstream);
    }
    if (x.caf_update_intrinsics_flag()) {
      x.miv_view_params_update_intrinsics() = MivViewParamsUpdateIntrinsics::decodeFrom(bitstream);
    }
    if (x.caf_update_depth_quantization_flag()) {
      x.miv_view_params_update_depth_quantization() =
          MivViewParamsUpdateDepthQuantization::decodeFrom(bitstream, vps);
    }
  }
  x.caf_extension_present_flag(bitstream.getFlag());

  if (x.caf_extension_present_flag()) {
    x.caf_extension_8bits(bitstream.readBits<uint8_t>(8));
  }
  if (x.caf_extension_8bits() != 0) {
    auto cafExtensionData = std::vector<bool>{};
    while (bitstream.moreRbspData()) {
      cafExtensionData.push_back(bitstream.getFlag());
    }
    x.cafExtensionData(std::move(cafExtensionData));
  }
  bitstream.rbspTrailingBits();

  return x;
}

void CommonAtlasFrameRBSP::encodeTo(std::ostream &stream, const V3cParameterSet &vps,
                                    unsigned maxCommonAtlasFrmOrderCntLsb) const {
  Common::OutputBitstream bitstream{stream};

  bitstream.putUExpGolomb(caf_atlas_adaptation_parameter_set_id());
  bitstream.putUVar(caf_frm_order_cnt_lsb(), maxCommonAtlasFrmOrderCntLsb);
  bitstream.putFlag(caf_irap_flag());

  if (caf_irap_flag()) {
    miv_view_params_list().encodeTo(bitstream, vps);
  } else {
    bitstream.putFlag(caf_update_extrinsics_flag());
    bitstream.putFlag(caf_update_intrinsics_flag());
    bitstream.putFlag(caf_update_depth_quantization_flag());

    if (caf_update_extrinsics_flag()) {
      miv_view_params_update_extrinsics().encodeTo(bitstream);
    }
    if (caf_update_intrinsics_flag()) {
      miv_view_params_update_intrinsics().encodeTo(bitstream);
    }
    if (caf_update_depth_quantization_flag()) {
      miv_view_params_update_depth_quantization().encodeTo(bitstream, vps);
    }
  }

  bitstream.putFlag(caf_extension_present_flag());

  if (caf_extension_present_flag()) {
    bitstream.writeBits(caf_extension_8bits(), 8);
  }
  if (caf_extension_8bits() != 0) {
    for (const auto bit : cafExtensionData()) {
      bitstream.putFlag(bit);
    }
  }
  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream
