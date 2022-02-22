/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#include <TMIV/Decoder/DecodeCommonAtlas.h>

#include <TMIV/Common/Decoder.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/SeiRBSP.h>

#include "NalUnitSemantics.h"

namespace TMIV::Decoder {
namespace {
class CommonAtlasDecoder final
    : public Common::Decoder<MivBitstream::NalUnit, CommonAtlasAccessUnit> {
public:
  using Base = Common::Decoder<MivBitstream::NalUnit, CommonAtlasAccessUnit>;

  CommonAtlasDecoder(Common::Source<MivBitstream::NalUnit> source,
                     MivBitstream::V3cParameterSet vps, PtlChecker::SharedChecker checker)
      : Base{std::move(source)}, m_vps{std::move(vps)}, m_checker{std::move(checker)} {}

protected:
  auto decodeSome() -> bool final {
    auto au = CommonAtlasAccessUnit{};

    if (!m_nu) {
      m_nu = pull();
    }
    if (!m_nu) {
      return false;
    }
    while (isAud(nut())) {
      m_nu = pull();
    }
    while (isPrefixNalUnit(nut())) {
      decodePrefixNalUnit(au);
      m_nu = pull();
    }

    VERIFY_V3CBITSTREAM(isCaf(nut()));
    decodeCafNalUnit(au);
    m_nu = pull();

    while (m_nu && isSuffixNalUnit(nut())) {
      decodeSuffixNalUnit(au);
      m_nu = pull();
    }
    while (m_nu && isEos(nut())) {
      m_nu = pull();
    }
    while (m_nu && isEob(nut())) {
      m_nu = pull();
    }

    push(au);
    return bool{m_nu};
  }

private:
  auto pull() -> std::optional<MivBitstream::NalUnit> {
    if (auto nu = Base::pull()) {
      m_checker->checkNuh(nu->nal_unit_header());
      return nu;
    }
    return std::nullopt;
  }

  [[nodiscard]] auto nut() const -> MivBitstream::NalUnitType {
    VERIFY_V3CBITSTREAM(m_nu);
    return m_nu->nal_unit_header().nal_unit_type();
  }

  void decodePrefixNalUnit(CommonAtlasAccessUnit &au) {
    std::istringstream stream{m_nu->rbsp()};

    switch (nut()) {
    case MivBitstream::NalUnitType::NAL_CASPS:
      return decodeCasps(stream);
    case MivBitstream::NalUnitType::NAL_PREFIX_ESEI:
    case MivBitstream::NalUnitType::NAL_PREFIX_NSEI:
      return decodeSei(au, stream);
    default:
      fmt::print("WARNING: Ignoring prefix NAL unit {}", nut());
    }
  }

  void decodeCafNalUnit(CommonAtlasAccessUnit &au) {
    std::istringstream stream{m_nu->rbsp()};

    VERIFY_MIVBITSTREAM(0 < m_maxCommonAtlasFrmOrderCntLsb);
    au.caf = MivBitstream::CommonAtlasFrameRBSP::decodeFrom(
        stream, m_nu->nal_unit_header(), m_caspsV, m_maxCommonAtlasFrmOrderCntLsb);

    m_checker->checkCaf(m_nu->nal_unit_header(), au.caf);

    const auto focLsb = au.caf.caf_common_atlas_frm_order_cnt_lsb();
    const auto irap = nut() == MivBitstream::NalUnitType::NAL_CAF_IDR;

    if (irap) {
      VERIFY_V3CBITSTREAM(focLsb == 0);
      m_foc = 0;
    } else {
      VERIFY_V3CBITSTREAM(focLsb < m_maxCommonAtlasFrmOrderCntLsb);
      while (m_foc % m_maxCommonAtlasFrmOrderCntLsb != focLsb) {
        ++m_foc;
      }
    }

    au.foc = m_foc;
    au.casps = caspsById(m_caspsV, au.caf.caf_common_atlas_sequence_parameter_set_id());
  }

  void decodeSuffixNalUnit(CommonAtlasAccessUnit &au) {
    std::istringstream stream{m_nu->rbsp()};

    switch (nut()) {
    case MivBitstream::NalUnitType::NAL_FD:
      return; // Ignore filler data
    case MivBitstream::NalUnitType::NAL_SUFFIX_ESEI:
    case MivBitstream::NalUnitType::NAL_SUFFIX_NSEI:
      return decodeSei(au, stream);
    default:
      fmt::print("WARNING: Ignoring suffix NAL unit {}", nut());
    }
  }

  void decodeCasps(std::istream &stream) {
    auto casps = MivBitstream::CommonAtlasSequenceParameterSetRBSP::decodeFrom(stream);

    const auto maxCommonAtlasFrmOrderCntLsb =
        1U << (casps.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4() + 4U);
    VERIFY_MIVBITSTREAM(m_maxCommonAtlasFrmOrderCntLsb == 0 ||
                        m_maxCommonAtlasFrmOrderCntLsb == maxCommonAtlasFrmOrderCntLsb);
    m_maxCommonAtlasFrmOrderCntLsb = maxCommonAtlasFrmOrderCntLsb;

    for (auto &storedCasps : m_caspsV) {
      if (storedCasps.casps_common_atlas_sequence_parameter_set_id() ==
          casps.casps_common_atlas_sequence_parameter_set_id()) {
        storedCasps = std::move(casps);
        return;
      }
    }
    return m_caspsV.push_back(casps);
  }

  static void decodeSei(CommonAtlasAccessUnit &au, std::istream &stream) {
    auto sei = MivBitstream::SeiRBSP::decodeFrom(stream);
    for (auto &message : sei.messages()) {
      decodeSeiMessage(au, message);
    }
  }

  static void decodeSeiMessage(CommonAtlasAccessUnit &au, const MivBitstream::SeiMessage &message) {
    std::istringstream messageStream{message.payload()};
    Common::InputBitstream bitstream{messageStream};

    switch (message.payloadType()) {
    case MivBitstream::PayloadType::geometry_upscaling_parameters:
      au.gup = MivBitstream::GeometryUpscalingParameters::decodeFrom(bitstream);
      return;
    case MivBitstream::PayloadType::viewing_space:
      au.vs = MivBitstream::ViewingSpace::decodeFrom(bitstream);
      return;
    case MivBitstream::PayloadType::viewport_camera_parameters:
      au.vcp = MivBitstream::ViewportCameraParameters::decodeFrom(bitstream);
      return;
    case MivBitstream::PayloadType::viewport_position:
      au.vp = MivBitstream::ViewportPosition::decodeFrom(bitstream);
      return;
    case MivBitstream::PayloadType::atlas_view_enabled:
      au.ave = MivBitstream::AtlasViewEnabled::decodeFrom(bitstream);
      return;
    default:
      fmt::print("WARNING: Ignoring SEI message {}", message.payloadType());
    }
  }

  MivBitstream::V3cParameterSet m_vps;
  PtlChecker::SharedChecker m_checker;

  std::optional<MivBitstream::NalUnit> m_nu;
  int32_t m_foc{-1};
  std::vector<MivBitstream::CommonAtlasSequenceParameterSetRBSP> m_caspsV;
  uint32_t m_maxCommonAtlasFrmOrderCntLsb{};
};
} // namespace

auto decodeCommonAtlas(Common::Source<MivBitstream::NalUnit> source,
                       MivBitstream::V3cParameterSet vps, PtlChecker::SharedChecker checker)
    -> Common::Source<CommonAtlasAccessUnit> {
  return [decoder = std::make_shared<CommonAtlasDecoder>(
              std::move(source), std::move(vps), std::move(checker))]() { return (*decoder)(); };
}
} // namespace TMIV::Decoder
