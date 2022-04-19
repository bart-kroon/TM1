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

#include <TMIV/Decoder/DecodeAtlas.h>

#include <TMIV/Common/Decoder.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/SeiRBSP.h>

#include "NalUnitSemantics.h"

#include <fmt/ostream.h>

#include <sstream>

namespace TMIV::Decoder {
namespace {
class AtlasDecoder final : public Common::Decoder<MivBitstream::NalUnit, AtlasAccessUnit> {
public:
  using Base = Common::Decoder<MivBitstream::NalUnit, AtlasAccessUnit>;

  AtlasDecoder(Common::Source<MivBitstream::NalUnit> source, const MivBitstream::V3cUnitHeader &vuh,
               MivBitstream::V3cParameterSet vps, PtlChecker::SharedChecker checker)
      : Base{std::move(source)}, m_vuh{vuh}, m_vps{std::move(vps)}, m_checker{std::move(checker)} {}

protected:
  auto decodeSome() -> bool final {
    auto au = AtlasAccessUnit{};

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

    VERIFY_V3CBITSTREAM(isAcl(nut()));
    decodeAclNalUnit(au);
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

  void decodePrefixNalUnit(AtlasAccessUnit &au) {
    std::istringstream stream{m_nu->rbsp()};

    switch (nut()) {
    case MivBitstream::NalUnitType::NAL_ASPS:
      return decodeAsps(stream);
    case MivBitstream::NalUnitType::NAL_AFPS:
      return decodeAfps(stream);
    case MivBitstream::NalUnitType::NAL_PREFIX_ESEI:
    case MivBitstream::NalUnitType::NAL_PREFIX_NSEI:
      return decodeSei(au, stream);
    default:
      fmt::print("WARNING: Ignoring prefix NAL unit {}.\n", nut());
    }
  }

  void decodeAclNalUnit(AtlasAccessUnit &au) {
    std::istringstream stream{m_nu->rbsp()};
    au.atl = MivBitstream::AtlasTileLayerRBSP::decodeFrom(stream, m_nu->nal_unit_header(), m_aspsV,
                                                          m_afpsV);
    m_checker->checkAtl(m_nu->nal_unit_header(), au.atl);

    const auto focLsb = au.atl.atlas_tile_header().ath_atlas_frm_order_cnt_lsb();
    const auto irap = MivBitstream::NalUnitType::NAL_IDR_W_RADL <= nut() &&
                      nut() <= MivBitstream::NalUnitType::NAL_RSV_IRAP_ACL_29;

    if (irap) {
      VERIFY_V3CBITSTREAM(focLsb == 0);
      m_foc = 0;
    } else {
      VERIFY_V3CBITSTREAM(focLsb < m_maxAtlasFrmOrderCntLsb);
      while (m_foc % m_maxAtlasFrmOrderCntLsb != focLsb) {
        ++m_foc;
      }
    }

    au.foc = m_foc;
    au.afps = afpsById(m_afpsV, au.atl.atlas_tile_header().ath_atlas_frame_parameter_set_id());
    au.asps = aspsById(m_aspsV, au.afps.afps_atlas_sequence_parameter_set_id());
  }

  void decodeSuffixNalUnit(AtlasAccessUnit &au) {
    std::istringstream stream{m_nu->rbsp()};

    switch (m_nu->nal_unit_header().nal_unit_type()) {
    case MivBitstream::NalUnitType::NAL_FD:
      return; // Ignore filler data
    case MivBitstream::NalUnitType::NAL_SUFFIX_ESEI:
    case MivBitstream::NalUnitType::NAL_SUFFIX_NSEI:
      return decodeSei(au, stream);
    default:
      fmt::print("WARNING: Ignoring suffix NAL unit {}\n", nut());
    }
  }

  void decodeAsps(std::istream &stream) {
    auto asps = MivBitstream::AtlasSequenceParameterSetRBSP::decodeFrom(stream, m_vuh, m_vps);

    m_checker->checkAsps(m_vuh.vuh_atlas_id(), asps);

    m_maxAtlasFrmOrderCntLsb = 1U << (asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4U);

    for (auto &x : m_aspsV) {
      if (x.asps_atlas_sequence_parameter_set_id() == asps.asps_atlas_sequence_parameter_set_id()) {
        VERIFY_V3CBITSTREAM(x == asps);
        return;
      }
    }
    return m_aspsV.push_back(asps);
  }

  void decodeAfps(std::istream &stream) {
    auto afps = MivBitstream::AtlasFrameParameterSetRBSP::decodeFrom(stream, m_aspsV);

    m_checker->checkAfps(afps);

    for (auto &x : m_afpsV) {
      if (x.afps_atlas_frame_parameter_set_id() == afps.afps_atlas_frame_parameter_set_id()) {
        x = std::move(afps);
        return;
      }
    }
    return m_afpsV.push_back(afps);
  }

  void decodeSei(AtlasAccessUnit &au, std::istream &stream) const {
    const auto sei = MivBitstream::SeiRBSP::decodeFrom(stream, nut());

    for (const auto &message : sei.messages()) {
      std::visit([&au](const auto &payload) { decodeSeiMessage(au, payload); },
                 message.seiPayload().payload);
    }
  }

  template <typename Payload>
  static void decodeSeiMessage([[maybe_unused]] AtlasAccessUnit &au,
                               [[maybe_unused]] const Payload &payload) {}

  static void decodeSeiMessage(AtlasAccessUnit &au,
                               const MivBitstream::GeometryAssistance &payload) {
    au.ga.push_back(payload);
  }

  MivBitstream::V3cUnitHeader m_vuh;
  MivBitstream::V3cParameterSet m_vps;
  PtlChecker::SharedChecker m_checker;

  std::optional<MivBitstream::NalUnit> m_nu;
  int32_t m_foc{-1};
  std::vector<MivBitstream::AtlasSequenceParameterSetRBSP> m_aspsV;
  std::vector<MivBitstream::AtlasFrameParameterSetRBSP> m_afpsV;
  uint32_t m_maxAtlasFrmOrderCntLsb{};
};
} // namespace

auto decodeAtlas(Common::Source<MivBitstream::NalUnit> source,
                 const MivBitstream::V3cUnitHeader &vuh, MivBitstream::V3cParameterSet vps,
                 PtlChecker::SharedChecker checker) -> Common::Source<AtlasAccessUnit> {
  return [decoder = std::make_shared<AtlasDecoder>(std::move(source), vuh, std::move(vps),
                                                   std::move(checker))]() { return (*decoder)(); };
}
} // namespace TMIV::Decoder
