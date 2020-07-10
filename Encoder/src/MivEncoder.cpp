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

#include <TMIV/Encoder/MivEncoder.h>

#include <TMIV/MivBitstream/FrameOrderCountRBSP.h>

#include <TMIV/MivBitstream/verify.h>

#include <sstream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::Encoder {
MivEncoder::MivEncoder(std::ostream &stream) : m_stream{stream} {
  m_ssvh.encodeTo(m_stream);
  m_stream.flush();
}

void MivEncoder::writeIvSequenceParams(const IvSequenceParams &ivSequenceParams) {
  m_ivs = ivSequenceParams;

  writeV3cUnit(VuhUnitType::V3C_VPS, 0, m_ivs.vps);
  writeV3cUnit(VuhUnitType::V3C_AD, specialAtlasId, specialAtlasSubBitstream());
}

void MivEncoder::writeIvAccessUnitParams(const IvAccessUnitParams &ivAccessUnitParams) {
  m_ivau = ivAccessUnitParams;

  for (uint8_t vai = 0; vai <= m_ivs.vps.vps_atlas_count_minus1(); ++vai) {
    writeV3cUnit(VuhUnitType::V3C_AD, vai, atlasSubBitstream(vai));
  }

  m_irap = false;
}

namespace {
const auto nuhAps = NalUnitHeader{NalUnitType::NAL_AAPS, 0, 1};
const auto nuhAsps = NalUnitHeader{NalUnitType::NAL_ASPS, 0, 1};
const auto nuhAfps = NalUnitHeader{NalUnitType::NAL_AFPS, 0, 1};
const auto nuhIdr = NalUnitHeader{NalUnitType::NAL_IDR_N_LP, 0, 1};
const auto nuhCra = NalUnitHeader{NalUnitType::NAL_CRA, 0, 1};
} // namespace

auto MivEncoder::specialAtlasSubBitstream() -> AtlasSubBitstream {
  auto asb = AtlasSubBitstream{m_ssnh};
  m_ivs.updateMvpl();
  writeNalUnit(asb, nuhAps, m_ivs.aaps, m_ivs.vps);
  return asb;
}

auto MivEncoder::atlasSubBitstream(std::uint8_t vai) -> AtlasSubBitstream {
  auto asb = AtlasSubBitstream{m_ssnh};

  auto vuh = V3cUnitHeader{VuhUnitType::V3C_AD};
  vuh.vuh_atlas_id(vai);

  const auto &aau = m_ivau.atlas[vai];
  const auto aspsV = vector<AtlasSequenceParameterSetRBSP>{aau.asps};
  const auto afpsV = vector<AtlasFrameParameterSetRBSP>{aau.afps};

  if (m_irap) {
    writeNalUnit(asb, nuhAsps, aau.asps, vuh, m_ivs.vps);
    writeNalUnit(asb, nuhAfps, aau.afps, vector<AtlasSequenceParameterSetRBSP>{aau.asps});
    writeNalUnit(asb, nuhIdr, atlasTileGroupLayer(vai), vuh, m_ivs.vps, aspsV, afpsV);
  } else {
    writeNalUnit(asb, nuhCra, atlasTileGroupLayer(vai), vuh, m_ivs.vps, aspsV, afpsV);
  }

  return asb;
}

auto MivEncoder::atlasTileGroupLayer(std::uint8_t vai) const -> AtlasTileLayerRBSP {
  auto patchData = AtlasTileDataUnit::Vector{};
  patchData.reserve(m_ivau.patchParamsList.size());

  const auto &aau = m_ivau.atlas[vai];

  const auto k = aau.asps.asps_log2_patch_packing_block_size();

  for (const auto &pp : m_ivau.patchParamsList) {
    if (pp.vuhAtlasId == vai) {
      auto pdu = PatchDataUnit{};

      VERIFY_MIVBITSTREAM(0 <= pp.pdu2dPos().x() && pp.pdu2dPos().x() <= UINT16_MAX);
      VERIFY_MIVBITSTREAM(0 <= pp.pdu2dPos().y() && pp.pdu2dPos().y() <= UINT16_MAX);
      VERIFY_MIVBITSTREAM(pp.pdu2dPos().x() % (1 << k) == 0);
      VERIFY_MIVBITSTREAM(pp.pdu2dPos().y() % (1 << k) == 0);
      pdu.pdu_2d_pos_x(uint16_t(pp.pdu2dPos().x()) >> k);
      pdu.pdu_2d_pos_y(uint16_t(pp.pdu2dPos().y()) >> k);

      VERIFY_MIVBITSTREAM(0 < pp.pdu2dSize().x() && pp.pdu2dSize().x() <= UINT16_MAX + 1);
      VERIFY_MIVBITSTREAM(0 < pp.pdu2dSize().y() && pp.pdu2dSize().y() <= UINT16_MAX + 1);
      VERIFY_MIVBITSTREAM(pp.pdu2dSize().x() % (1 << k) == 0);
      VERIFY_MIVBITSTREAM(pp.pdu2dSize().y() % (1 << k) == 0);
      pdu.pdu_2d_size_x_minus1(uint16_t(pp.pdu2dSize().x() - 1) >> k);
      pdu.pdu_2d_size_y_minus1(uint16_t(pp.pdu2dSize().y() - 1) >> k);

      pdu.pdu_view_pos_x(pp.pduViewPos().x());
      pdu.pdu_view_pos_y(pp.pduViewPos().y());

      VERIFY_MIVBITSTREAM(pp.pduDepthStart() % (1 << aau.ath.ath_pos_min_z_quantizer()) == 0);
      pdu.pdu_depth_start(pp.pduDepthStart() >> aau.ath.ath_pos_min_z_quantizer());

      if (pp.pduDepthEnd()) {
        const auto ath_pos_delta_max_z_quantizer = aau.ath.ath_pos_delta_max_z_quantizer();
        VERIFY_MIVBITSTREAM(*pp.pduDepthEnd() % (1 << ath_pos_delta_max_z_quantizer) == 0);
        pdu.pdu_depth_end(*pp.pduDepthEnd() >> ath_pos_delta_max_z_quantizer);
      }

      pdu.pdu_orientation_index(pp.pduOrientationIndex());
      pdu.pdu_projection_id(pp.pduViewId());

      if (pp.pduEntityId()) {
        pdu.pdu_miv_extension().pdu_entity_id(*pp.pduEntityId());
      }
      if (pp.pduDepthOccMapThreshold()) {
        pdu.pdu_miv_extension().pdu_depth_occ_threshold(*pp.pduDepthOccMapThreshold());
      }
      patchData.emplace_back(AtduPatchMode::I_INTRA, pdu);
    }
  }

  return AtlasTileLayerRBSP{aau.ath, AtlasTileDataUnit{patchData}};
}

template <typename Payload>
void MivEncoder::writeV3cUnit(VuhUnitType vut, uint8_t vai, Payload &&payload) {
  auto vuh = V3cUnitHeader{vut};
  if (vai != 0) {
    vuh.vuh_atlas_id(vai);
  }
  const auto vu = V3cUnit{vuh, forward<Payload>(payload)};

  ostringstream substream;
  vu.encodeTo(substream);

  const auto ssvu = SampleStreamV3cUnit{substream.str()};
  ssvu.encodeTo(m_stream, m_ssvh);
}

template <typename Payload, typename... Args>
void MivEncoder::writeNalUnit(AtlasSubBitstream &asb, NalUnitHeader nuh, Payload &&payload,
                              Args &&... args) {
  ostringstream substream1;
  payload.encodeTo(substream1, forward<Args>(args)...);
  asb.nal_units().emplace_back(nuh, substream1.str());
}
} // namespace TMIV::Encoder
