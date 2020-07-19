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

#include <TMIV/MivBitstream/verify.h>

#include <sstream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::Encoder {
MivEncoder::MivEncoder(std::ostream &stream) : m_stream{stream} {
  m_ssvh.encodeTo(m_stream);
  m_stream.flush();
}

void MivEncoder::writeAccessUnit(const EncoderParams &params) {
  m_params = params;

  if (m_irap) {
    writeV3cUnit(VuhUnitType::V3C_VPS, 0, m_params.vps);
    m_log2MaxFrmOrderCntLsbMinus4 = m_params.aaps.aaps_log2_max_atlas_frame_order_cnt_lsb_minus4();
  }

  m_frmOrderCntLsb = m_params.atlas.front().ath.ath_atlas_frm_order_cnt_lsb();
  VERIFY_MIVBITSTREAM(m_frmOrderCntLsb < maxFrmOrderCntLsb());

  if (m_irap || m_viewParamsList != params.viewParamsList) {
    writeV3cUnit(VuhUnitType::V3C_CAD, 0, commonAtlasSubBitstream());
    m_viewParamsList = params.viewParamsList;
  }

  for (uint8_t vai = 0; vai <= m_params.vps.vps_atlas_count_minus1(); ++vai) {
    // Clause 7.4.5.3.2 of V-PCC DIS d85 [N19329]: AXPS regardless of atlas ID (and temporal ID)
    // share the same value space for AXPS ID
    auto &aau = m_params.atlas[vai];
    aau.asps.asps_atlas_sequence_parameter_set_id(vai);
    aau.afps.afps_atlas_frame_parameter_set_id(vai);
    aau.afps.afps_atlas_sequence_parameter_set_id(vai);
    aau.ath.ath_atlas_frame_parameter_set_id(vai);

    writeV3cUnit(VuhUnitType::V3C_AD, vai, atlasSubBitstream(vai));
  }

  m_irap = false;
}

namespace {
const auto nuhAaps = NalUnitHeader{NalUnitType::NAL_AAPS, 0, 1};
const auto nuhAsps = NalUnitHeader{NalUnitType::NAL_ASPS, 0, 1};
const auto nuhAfps = NalUnitHeader{NalUnitType::NAL_AFPS, 0, 1};
const auto nuhIdr = NalUnitHeader{NalUnitType::NAL_IDR_N_LP, 0, 1};
const auto nuhCra = NalUnitHeader{NalUnitType::NAL_CRA, 0, 1};
const auto nuhCaf = NalUnitHeader{NalUnitType::NAL_CAF, 0, 1};
} // namespace

auto MivEncoder::commonAtlasSubBitstream() -> AtlasSubBitstream {
  auto asb = AtlasSubBitstream{m_ssnh};

  if (m_irap) {
    writeNalUnit(asb, nuhAaps, m_params.aaps);
  }

  writeNalUnit(asb, nuhCaf, commonAtlasFrame(), m_params.vps, maxFrmOrderCntLsb());
  return asb;
}

auto MivEncoder::commonAtlasFrame() const -> CommonAtlasFrameRBSP {
  auto caf = CommonAtlasFrameRBSP{};

  const auto mode = mvpUpdateMode();
  caf.caf_atlas_adaptation_parameter_set_id(0)
      .caf_frm_order_cnt_lsb(m_frmOrderCntLsb)
      .caf_miv_view_params_list_update_mode(mode);

  if (mode == MvpUpdateMode::VPL_INITLIST) {
    caf.miv_view_params_list() = mivViewParamsList();
  } else {
    if (mode == MvpUpdateMode::VPL_UPD_EXT || mode == MvpUpdateMode::VPL_ALL) {
      caf.miv_view_params_update_extrinsics() = mivViewParamsUpdateExtrinsics();
    }
    if (mode == MvpUpdateMode::VPL_UPD_INT || mode == MvpUpdateMode::VPL_ALL) {
      caf.miv_view_params_update_intrinsics() = mivViewParamsUpdateIntrinsics();
    }
    if (mode == MvpUpdateMode::VPL_UPD_DQ || mode == MvpUpdateMode::VPL_ALL) {
      caf.miv_view_params_update_depth_quantization() = mivViewParamsUpdateDepthQuantization();
    }
  }
  return caf;
}

auto MivEncoder::mvpUpdateMode() const -> MvpUpdateMode {
  if (m_irap) {
    return MvpUpdateMode::VPL_INITLIST;
  }
  auto updExt = false;
  auto updInt = false;
  auto updDq = false;
  VERIFY_MIVBITSTREAM(m_viewParamsList.size() == m_params.viewParamsList.size());
  for (size_t i = 0; i < m_viewParamsList.size(); ++i) {
    updExt = updExt || m_viewParamsList[i].ce != m_params.viewParamsList[i].ce;
    updInt = updInt || m_viewParamsList[i].ci != m_params.viewParamsList[i].ci;
    updDq = updDq || m_viewParamsList[i].dq != m_params.viewParamsList[i].dq;
  }
  if (int(updExt) + int(updInt) + int(updDq) > 1) {
    return MvpUpdateMode::VPL_ALL;
  }
  if (updExt) {
    return MvpUpdateMode::VPL_UPD_EXT;
  }
  if (updInt) {
    return MvpUpdateMode::VPL_UPD_INT;
  }
  if (updDq) {
    return MvpUpdateMode::VPL_UPD_DQ;
  }
  MIVBITSTREAM_ERROR("It is not possible to have a CAF that does not update view parameters.");
}

auto MivEncoder::mivViewParamsList() const -> MivViewParamsList {
  auto mvpl = MivViewParamsList{};
  auto &vpl = m_params.viewParamsList;

  assert(!vpl.empty());
  mvpl.mvp_num_views_minus1(uint16_t(vpl.size() - 1));
  mvpl.mvp_intrinsic_params_equal_flag(
      all_of(vpl.begin(), vpl.end(), [&](const auto &x) { return x.ci == vpl.front().ci; }));
  mvpl.mvp_depth_quantization_params_equal_flag(
      all_of(vpl.begin(), vpl.end(), [&](const auto &x) { return x.dq == vpl.front().dq; }));
  mvpl.mvp_pruning_graph_params_present_flag(vpl.front().pp.has_value());

  for (uint16_t i = 0; i <= mvpl.mvp_num_views_minus1(); ++i) {
    const auto &vp = vpl[i];
    mvpl.camera_extrinsics(i) = vp.ce;

    if (i == 0 || !mvpl.mvp_intrinsic_params_equal_flag()) {
      mvpl.camera_intrinsics(i) = vp.ci;
    }
    if (i == 0 || !mvpl.mvp_depth_quantization_params_equal_flag()) {
      mvpl.depth_quantization(i) = vp.dq;
    }
    assert(vp.pp.has_value() == mvpl.mvp_pruning_graph_params_present_flag());
    if (vp.pp.has_value()) {
      mvpl.pruning_parent(i) = *vp.pp;
    }
  }

  mvpl.mvp_num_views_minus1(uint16_t(m_params.viewParamsList.size() - 1));
  for (uint8_t a = 0; a <= m_params.vps.vps_atlas_count_minus1(); ++a) {
    for (uint16_t v = 0; v <= mvpl.mvp_num_views_minus1(); ++v) {
      mvpl.mvp_view_enabled_in_atlas_flag(a, v, true);
      mvpl.mvp_view_complete_in_atlas_flag(a, v, m_params.viewParamsList[v].isBasicView);
    }
  }
  mvpl.mvp_explicit_view_id_flag(true);
  for (uint16_t v = 0; v <= mvpl.mvp_num_views_minus1(); ++v) {
    mvpl.mvp_view_id(v, v);
  }

  return mvpl;
}

auto MivEncoder::mivViewParamsUpdateExtrinsics() const -> MivViewParamsUpdateExtrinsics {
  auto mvpue = MivViewParamsUpdateExtrinsics{};
  auto viewIdx = vector<uint16_t>{};
  for (size_t v = 0; v < m_viewParamsList.size(); ++v) {
    if (m_viewParamsList[v].ce != m_params.viewParamsList[v].ce) {
      viewIdx.push_back(uint16_t(v));
    }
  }
  VERIFY_MIVBITSTREAM(!viewIdx.empty());
  mvpue.mvpue_num_view_updates_minus1(uint16_t(viewIdx.size() - 1));
  for (uint16_t i = 0; i <= mvpue.mvpue_num_view_updates_minus1(); ++i) {
    mvpue.mvpue_view_idx(i, viewIdx[i]);
    mvpue.camera_extrinsics(i) = m_params.viewParamsList[viewIdx[i]].ce;
  }
  return mvpue;
}

auto MivEncoder::mivViewParamsUpdateIntrinsics() const -> MivViewParamsUpdateIntrinsics {
  auto mvpui = MivViewParamsUpdateIntrinsics{};
  auto viewIdx = vector<uint16_t>{};
  for (size_t v = 0; v < m_viewParamsList.size(); ++v) {
    if (m_viewParamsList[v].ci != m_params.viewParamsList[v].ci) {
      viewIdx.push_back(uint16_t(v));
    }
  }
  VERIFY_MIVBITSTREAM(!viewIdx.empty());
  mvpui.mvpui_num_view_updates_minus1(uint16_t(viewIdx.size() - 1));
  for (uint16_t i = 0; i <= mvpui.mvpui_num_view_updates_minus1(); ++i) {
    mvpui.mvpui_view_idx(i, viewIdx[i]);
    mvpui.camera_intrinsics(i) = m_params.viewParamsList[viewIdx[i]].ci;
  }
  return mvpui;
}

auto MivEncoder::mivViewParamsUpdateDepthQuantization() const
    -> MivViewParamsUpdateDepthQuantization {
  auto mvpudq = MivViewParamsUpdateDepthQuantization{};
  auto viewIdx = vector<uint16_t>{};
  for (size_t v = 0; v < m_viewParamsList.size(); ++v) {
    if (m_viewParamsList[v].dq != m_params.viewParamsList[v].dq) {
      viewIdx.push_back(uint16_t(v));
    }
  }
  VERIFY_MIVBITSTREAM(!viewIdx.empty());
  mvpudq.mvpudq_num_view_updates_minus1(uint16_t(viewIdx.size() - 1));
  for (uint16_t i = 0; i <= mvpudq.mvpudq_num_view_updates_minus1(); ++i) {
    mvpudq.mvpudq_view_idx(i, viewIdx[i]);
    mvpudq.depth_quantization(i) = m_params.viewParamsList[viewIdx[i]].dq;
  }
  return mvpudq;
}

auto MivEncoder::atlasSubBitstream(std::uint8_t vai) -> AtlasSubBitstream {
  auto asb = AtlasSubBitstream{m_ssnh};

  auto vuh = V3cUnitHeader{VuhUnitType::V3C_AD};
  vuh.vuh_atlas_id(vai);

  const auto &aau = m_params.atlas[vai];

  if (m_irap) {
    VERIFY_MIVBITSTREAM(m_log2MaxFrmOrderCntLsbMinus4 ==
                        aau.asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4());
    writeNalUnit(asb, nuhAsps, aau.asps, vuh, m_params.vps);
    writeNalUnit(asb, nuhAfps, aau.afps, vector<AtlasSequenceParameterSetRBSP>{aau.asps});
  }

  const auto aspsV = vector<AtlasSequenceParameterSetRBSP>{aau.asps};
  const auto afpsV = vector<AtlasFrameParameterSetRBSP>{aau.afps};
  writeNalUnit(asb, m_irap ? nuhIdr : nuhCra, atlasTileGroupLayer(vai), vuh, m_params.vps, aspsV,
               afpsV);

  return asb;
}

auto MivEncoder::atlasTileGroupLayer(std::uint8_t vai) const -> AtlasTileLayerRBSP {
  auto patchData = AtlasTileDataUnit::Vector{};
  patchData.reserve(m_params.patchParamsList.size());

  const auto &aau = m_params.atlas[vai];

  const auto k = aau.asps.asps_log2_patch_packing_block_size();

  for (const auto &pp : m_params.patchParamsList) {
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

  auto x = AtlasTileLayerRBSP{};
  x.atlas_tile_header() = aau.ath;
  x.atlas_tile_data_unit() = AtlasTileDataUnit{patchData};
  return x;
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
