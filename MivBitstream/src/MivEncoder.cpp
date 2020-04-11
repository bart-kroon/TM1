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

#include <TMIV/MivBitstream/MivEncoder.h>

#include "verify.h"

#include <iostream>
#include <sstream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {

MivEncoder::MivEncoder(std::ostream &stream) : m_stream{stream} {
  cout << "=== Sample stream V-PCC header " << string(100 - 31, '=') << '\n'
       << m_ssvh << string(100, '=') << "\n"
       << endl;

  m_ssvh.encodeTo(m_stream);
  m_stream.flush();
}

void MivEncoder::writeIvSequenceParams(const IvSequenceParams &ivSequenceParams) {
  m_ivs = ivSequenceParams;

  writeVpccUnit(VuhUnitType::VPCC_VPS, 0, m_ivs.vps);
  writeVpccUnit(VuhUnitType::VPCC_AD, specialAtlasId, specialAtlasSubBitstream());
}

void MivEncoder::writeIvAccessUnitParams(const IvAccessUnitParams &ivAccessUnitParams,
                                         int intraPeriodFrameCount) {
  m_ivau = ivAccessUnitParams;

  if (m_writeNonAcl) {
    for (uint8_t vai = 0; vai <= m_ivs.vps.vps_atlas_count_minus1(); ++vai) {
      writeVpccUnit(VuhUnitType::VPCC_AD, vai, nonAclAtlasSubBitstream(vai));
    }
  }

  for (uint8_t vai = 0; vai <= m_ivs.vps.vps_atlas_count_minus1(); ++vai) {
    writeVpccUnit(VuhUnitType::VPCC_AD, vai, aclAtlasSubBitstream(vai, intraPeriodFrameCount));
  }

  m_writeNonAcl = false;
}

namespace {
const auto nuhAps = NalUnitHeader{NalUnitType::NAL_APS, 0, 1};
const auto nuhAsps = NalUnitHeader{NalUnitType::NAL_ASPS, 0, 1};
const auto nuhAfps = NalUnitHeader{NalUnitType::NAL_AFPS, 0, 1};
const auto nuhIdr = NalUnitHeader{NalUnitType::NAL_IDR_N_LP, 0, 1};
const auto nuhCra = NalUnitHeader{NalUnitType::NAL_CRA, 0, 1};
const auto nuhSkip = NalUnitHeader{NalUnitType::NAL_SKIP, 0, 2};
} // namespace

auto MivEncoder::specialAtlasSubBitstream() -> AtlasSubBitstream {
  auto asb = AtlasSubBitstream{m_ssnh};
  writeNalUnit(asb, nuhAps, adaptationParameterSet());
  return asb;
}

auto MivEncoder::nonAclAtlasSubBitstream(std::uint8_t vai) -> AtlasSubBitstream {
  auto asb = AtlasSubBitstream{m_ssnh};

  auto vuh = VpccUnitHeader{VuhUnitType::VPCC_AD};
  vuh.vuh_atlas_id(vai);

  const auto &aau = m_ivau.atlas[vai];

  writeNalUnit(asb, nuhAsps, aau.asps, vuh, m_ivs.vps);
  writeNalUnit(asb, nuhAfps, aau.afps, vector<AtlasSequenceParameterSetRBSP>{aau.asps});
  return asb;
}

auto MivEncoder::aclAtlasSubBitstream(std::uint8_t vai, int intraPeriodFrameCount)
    -> AtlasSubBitstream {
  auto asb = AtlasSubBitstream{m_ssnh};

  auto vuh = VpccUnitHeader{VuhUnitType::VPCC_AD};
  vuh.vuh_atlas_id(vai);

  const auto &aau = m_ivau.atlas[vai];
  const auto aspsV = vector<AtlasSequenceParameterSetRBSP>{aau.asps};
  const auto afpsV = vector<AtlasFrameParameterSetRBSP>{aau.afps};

  const auto nuh = m_writeNonAcl ? nuhIdr : nuhCra;
  writeNalUnit(asb, nuh, atlasTileGroupLayer(vai), vuh, m_ivs.vps, aspsV, afpsV);

  for (int frameId = 1; frameId < intraPeriodFrameCount; ++frameId) {
    writeNalUnit(asb, nuhSkip, skipAtlasTileGroupLayer(), vuh, m_ivs.vps, aspsV, afpsV);
  }

  return asb;
}

auto MivEncoder::adaptationParameterSet() const -> AdaptationParameterSetRBSP {
  auto x = AdaptationParameterSetRBSP{};

  const auto &vpl = m_ivs.viewParamsList;

  auto &mvp = x.aps_miv_view_params_list_present_flag(true)
                  .aps_miv_view_params_list_update_mode(MvpUpdateMode::VPL_INITLIST)
                  .miv_view_params_list();

  VERIFY_MIVBITSTREAM(!vpl.empty());
  mvp.mvp_num_views_minus1(uint16_t(vpl.size() - 1));

  mvp.mvp_intrinsic_params_equal_flag(
      all_of(vpl.begin(), vpl.end(), [&vpl](const auto &x) { return x.ci == vpl.front().ci; }));

  mvp.mvp_depth_quantization_params_equal_flag(
      all_of(vpl.begin(), vpl.end(), [&vpl](const auto &x) { return x.dq == vpl.front().dq; }));

  mvp.mvp_pruning_graph_params_present_flag(vpl.front().pc.has_value());

  for (uint16_t viewId = 0; viewId <= mvp.mvp_num_views_minus1(); ++viewId) {
    const auto &vp = vpl[viewId];

    mvp.camera_extrinsics(viewId) = vp.ce;

    if (viewId == 0 || !mvp.mvp_intrinsic_params_equal_flag()) {
      mvp.camera_intrinsics(viewId) = vp.ci;
    }

    if (viewId == 0 || !mvp.mvp_depth_quantization_params_equal_flag()) {
      mvp.depth_quantization(viewId) = vp.dq;
    }

    VERIFY_MIVBITSTREAM(vp.pc.has_value() == mvp.mvp_pruning_graph_params_present_flag());
    if (vp.pc.has_value()) {
      mvp.pruning_children(viewId) = *vp.pc;
    }
  }

  return x;
}

auto MivEncoder::atlasTileGroupLayer(std::uint8_t vai) const -> AtlasTileGroupLayerRBSP {
  auto patchData = AtlasTileGroupDataUnit::Vector{};
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

      pdu.pdu_depth_start(pp.pduDepthStart());
      if (pp.pduDepthEnd()) {
        pdu.pdu_depth_end(*pp.pduDepthEnd());
      }

      pdu.pdu_orientation_index(pp.pduOrientationIndex());
      pdu.pdu_view_id(pp.pduViewId());

      if (pp.pduEntityId()) {
        pdu.pdu_entity_id(*pp.pduEntityId());
      }

      if (pp.pduDepthOccMapThreshold()) {
        pdu.pdu_depth_occ_map_threshold(*pp.pduDepthOccMapThreshold());
      }

      patchData.emplace_back(AtgduPatchMode::I_INTRA, pdu);
    }
  }

  return AtlasTileGroupLayerRBSP{aau.atgh, AtlasTileGroupDataUnit{patchData}};
}

auto MivEncoder::skipAtlasTileGroupLayer() -> AtlasTileGroupLayerRBSP {
  auto atgh = AtlasTileGroupHeader{};
  atgh.atgh_type(AtghType::SKIP_TILE_GRP);

  return AtlasTileGroupLayerRBSP{atgh};
}

template <typename Payload>
void MivEncoder::writeVpccUnit(VuhUnitType vut, uint8_t vai, Payload &&payload) {
  auto vuh = VpccUnitHeader{vut};
  if (vai != 0) {
    vuh.vuh_atlas_id(vai);
  }
  const auto vu = VpccUnit{vuh, forward<Payload>(payload)};

  ostringstream substream;
  vu.encodeTo(substream, {m_ivs.vps});

  const auto ssvu = SampleStreamVpccUnit{substream.str()};
  ssvu.encodeTo(m_stream, m_ssvh);
  cout << "\n=== V-PCC unit " << string(100 - 15, '=') << '\n'
       << ssvu << vu << m_nalUnitLog.str() << string(100, '=') << "\n"
       << endl;

  m_nalUnitLog.str("");
}

template <typename Payload, typename... Args>
void MivEncoder::writeNalUnit(AtlasSubBitstream &asb, NalUnitHeader nuh, Payload &&payload,
                              Args &&... args) {
  ostringstream substream1;
  payload.encodeTo(substream1, forward<Args>(args)...);
  asb.nal_units().emplace_back(nuh, substream1.str());
  m_nalUnitLog << "--- NAL unit " << string(100 - 13, '-') << '\n'
               << asb.nal_units().back() << payload;
}
} // namespace TMIV::MivBitstream
