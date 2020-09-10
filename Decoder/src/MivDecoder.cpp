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

#include <TMIV/Decoder/MivDecoder.h>

#include <TMIV/MivBitstream/verify.h>

#include <ctime>
#include <iostream>
#include <utility>

namespace TMIV::Decoder {
MivDecoder::MivDecoder(V3cUnitSource source) : m_inputBuffer{std::move(source)} {}

MivDecoder::~MivDecoder() {
  if (m_totalOccVideoDecodingTime > 0.) {
    std::cout << "Total ocupancy video sub bitstream decoding time: " << m_totalOccVideoDecodingTime
              << " s\n";
  }
  if (m_totalGeoVideoDecodingTime > 0.) {
    std::cout << "Total geometry video sub bitstream decoding time: " << m_totalGeoVideoDecodingTime
              << " s\n";
  }
  if (m_totalAttrVideoDecodingTime > 0.) {
    std::cout << "Total attribute video sub bitstream decoding time: "
              << m_totalAttrVideoDecodingTime << " s\n";
  }
}

void MivDecoder::setOccFrameServer(OccFrameServer value) { m_occFrameServer = move(value); }

void MivDecoder::setGeoFrameServer(GeoFrameServer value) { m_geoFrameServer = move(value); }

void MivDecoder::setAttrFrameServer(AttrFrameServer value) { m_attrFrameServer = move(value); }

auto MivDecoder::operator()() -> std::optional<AccessUnit> {
  m_au.irap = expectIrap();

  if (m_au.irap && !decodeVps()) {
    return {};
  }

  ++m_au.foc;

  if (!m_commonAtlasAu || m_commonAtlasAu->foc < m_au.foc) {
    m_commonAtlasAu = (*m_commonAtlasDecoder)();
  }
  if (m_commonAtlasAu && m_commonAtlasAu->foc == m_au.foc) {
    decodeCommonAtlas();
  }

  for (uint8_t j = 0; j <= m_au.vps.vps_atlas_count_minus1(); ++j) {
    if (!m_atlasAu[j] || m_atlasAu[j]->foc < m_au.foc) {
      m_atlasAu[j] = (*m_atlasDecoder[j])();
    }
    if (m_atlasAu[j] && m_atlasAu[j]->foc == m_au.foc) {
      decodeAtlas(j);
    }
  }

  auto result = std::array{false, false};

  for (uint8_t j = 0; j <= m_au.vps.vps_atlas_count_minus1(); ++j) {
    if (m_au.vps.vps_occupancy_video_present_flag(j)) {
      result[decodeOccVideo(j)] = true;
    }
  }
  for (uint8_t j = 0; j <= m_au.vps.vps_atlas_count_minus1(); ++j) {
    if (m_au.vps.vps_geometry_video_present_flag(j)) {
      result[decodeGeoVideo(j)] = true;
    }
  }
  for (uint8_t j = 0; j <= m_au.vps.vps_atlas_count_minus1(); ++j) {
    if (m_au.vps.vps_attribute_video_present_flag(j)) {
      result[decodeAttrVideo(j)] = true;
    }
  }

  if (result[false] && result[true]) {
    throw std::runtime_error("One of the video streams is truncated");
  }
  if (result[true]) {
    // TODO(BK): This copies the video frames.
    return m_au;
  }
  return {};
}

auto MivDecoder::expectIrap() const -> bool { return !m_commonAtlasDecoder; }

auto MivDecoder::decodeVps() -> bool {
  auto vu = m_inputBuffer(MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_VPS});
  if (!vu) {
    return false;
  }
  m_au.vps = vu->v3c_payload().v3c_parameter_set();

  checkCapabilities();

  auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_CAD};
  m_commonAtlasDecoder = std::make_unique<CommonAtlasDecoder>(
      [this, vuh]() { return m_inputBuffer(vuh); }, m_au.vps, m_au.foc);

  m_atlasDecoder.clear();
  m_atlasAu.assign(m_au.vps.vps_atlas_count_minus1() + size_t(1), {});
  m_au.atlas.clear();
  m_occVideoDecoder.clear();
  m_geoVideoDecoder.clear();
  m_attrVideoDecoder.clear();

  for (uint8_t j = 0; j <= m_au.vps.vps_atlas_count_minus1(); ++j) {
    auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_AD};
    vuh.vuh_atlas_id(m_au.vps.vps_atlas_id(j));
    m_atlasDecoder.push_back(std::make_unique<AtlasDecoder>(
        [this, vuh]() { return m_inputBuffer(vuh); }, vuh, m_au.vps, m_au.foc));
    m_au.atlas.emplace_back();

    if (m_au.vps.vps_occupancy_video_present_flag(j)) {
      auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_OVD};
      vuh.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id())
          .vuh_atlas_id(m_au.vps.vps_atlas_id(j));
      m_occVideoDecoder.push_back(startVideoDecoder(vuh, m_totalOccVideoDecodingTime));
    } else {
      m_occVideoDecoder.push_back(nullptr);
    }

    if (m_au.vps.vps_geometry_video_present_flag(j)) {
      auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_GVD};
      vuh.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id())
          .vuh_atlas_id(m_au.vps.vps_atlas_id(j));
      m_geoVideoDecoder.push_back(startVideoDecoder(vuh, m_totalGeoVideoDecodingTime));
    }

    if (m_au.vps.vps_attribute_video_present_flag(j)) {
      auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_AVD};
      vuh.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id())
          .vuh_atlas_id(m_au.vps.vps_atlas_id(j));
      m_attrVideoDecoder.push_back(startVideoDecoder(vuh, m_totalAttrVideoDecodingTime));
    }
  }

  return true;
}

void MivDecoder::checkCapabilities() const {
  VERIFY_MIVBITSTREAM(m_au.vps.vps_miv_extension_flag());

  for (uint8_t j = 0; j <= m_au.vps.vps_atlas_count_minus1(); ++j) {
    VERIFY_MIVBITSTREAM(!m_au.vps.vps_auxiliary_video_present_flag(j));
    // TODO(BK): Add more constraints (map count, attribute count, EOM, etc.)
  }
}

auto MivDecoder::startVideoDecoder(const MivBitstream::V3cUnitHeader &vuh, double &totalTime)
    -> std::unique_ptr<VideoDecoder::VideoServer> {
  std::string data;
  while (auto vu = m_inputBuffer(vuh)) {
    // TODO(BK): Let the video decoder pull V3C units. This implementation assumes the bitstream is
    // short enough to fit in memory. The reason for this shortcut is that the change requires
    // parsing of the Annex B byte stream, which can be easily done but it requires an additional
    // implementation effort.
    data += vu->v3c_payload().video_sub_bitstream().data();
  }
  if (data.empty()) {
    return {}; // Out-of-band?
  }

  const double t0 = std::clock();
  auto server = std::make_unique<VideoDecoder::VideoServer>(
      VideoDecoder::IVideoDecoder::create(
          m_au.vps.profile_tier_level().ptl_profile_codec_group_idc()),
      data);
  server->wait();
  totalTime += (std::clock() - t0) / CLOCKS_PER_SEC;
  return server;
}

void MivDecoder::decodeCommonAtlas() {
  decodeViewParamsList();

  for (const auto &sei : m_commonAtlasAu->prefixNSei) {
    if (sei.payloadType() == MivBitstream::PayloadType::geometry_upscaling_parameters) {
      std::istringstream stream{sei.payload()};
      Common::InputBitstream bitstream{stream};
      m_au.gup = MivBitstream::GeometryUpscalingParameters::decodeFrom(bitstream);
    }
  }
}

void MivDecoder::decodeViewParamsList() {
  switch (m_commonAtlasAu->caf.caf_miv_view_params_list_update_mode()) {
  case MivBitstream::MvpUpdateMode::VPL_INITLIST:
    decodeMvpl(m_commonAtlasAu->caf.miv_view_params_list());
    break;
  case MivBitstream::MvpUpdateMode::VPL_UPD_EXT:
    decodeMvpue(m_commonAtlasAu->caf.miv_view_params_update_extrinsics());
    break;
  case MivBitstream::MvpUpdateMode::VPL_UPD_INT:
    decodeMvpui(m_commonAtlasAu->caf.miv_view_params_update_intrinsics());
    break;
  case MivBitstream::MvpUpdateMode::VPL_UPD_DQ:
    decodeMvpudq(m_commonAtlasAu->caf.miv_view_params_update_depth_quantization());
    break;
  case MivBitstream::MvpUpdateMode::VPL_ALL:
    decodeMvpue(m_commonAtlasAu->caf.miv_view_params_update_extrinsics());
    decodeMvpui(m_commonAtlasAu->caf.miv_view_params_update_intrinsics());
    decodeMvpudq(m_commonAtlasAu->caf.miv_view_params_update_depth_quantization());
    break;
  default:
    MIVBITSTREAM_ERROR("Unknown MVP update mode");
  }

  if (m_commonAtlasAu->aaps.aaps_miv_extension_flag()) {
    const auto &aame = m_commonAtlasAu->aaps.aaps_miv_extension();
    if (aame.aame_vui_params_present_flag()) {
      const auto &vui = aame.vui_parameters();
      VERIFY_MIVBITSTREAM(!m_au.vui || *m_au.vui == vui);
      m_au.vui = vui;
    }
  }
}

void MivDecoder::decodeMvpl(const MivBitstream::MivViewParamsList &mvpl) {
  m_au.viewParamsList.assign(mvpl.mvp_num_views_minus1() + size_t(1), {});

  for (uint16_t viewId = 0; viewId <= mvpl.mvp_num_views_minus1(); ++viewId) {
    m_au.viewParamsList[viewId].ce = mvpl.camera_extrinsics(viewId);
    m_au.viewParamsList[viewId].ci = mvpl.camera_intrinsics(viewId);
    m_au.viewParamsList[viewId].dq = mvpl.depth_quantization(viewId);

    if (mvpl.mvp_pruning_graph_params_present_flag()) {
      m_au.viewParamsList[viewId].pp = mvpl.pruning_parent(viewId);
    }
  }
}

void MivDecoder::decodeMvpue(const MivBitstream::MivViewParamsUpdateExtrinsics &mvpue) {
  for (uint16_t i = 0; i <= mvpue.mvpue_num_view_updates_minus1(); ++i) {
    m_au.viewParamsList[mvpue.mvpue_view_idx(i)].ce = mvpue.camera_extrinsics(i);
  }
}

void MivDecoder::decodeMvpui(const MivBitstream::MivViewParamsUpdateIntrinsics &mvpui) {
  for (uint16_t i = 0; i <= mvpui.mvpui_num_view_updates_minus1(); ++i) {
    m_au.viewParamsList[mvpui.mvpui_view_idx(i)].ci = mvpui.camera_intrinsics(i);
  }
}

void MivDecoder::decodeMvpudq(const MivBitstream::MivViewParamsUpdateDepthQuantization &mvpudq) {
  for (uint16_t i = 0; i <= mvpudq.mvpudq_num_view_updates_minus1(); ++i) {
    m_au.viewParamsList[mvpudq.mvpudq_view_idx(i)].dq = mvpudq.depth_quantization(i);
  }
}

void MivDecoder::decodeAtlas(uint8_t j) {
  m_au.atlas[j].asps = m_atlasAu[j]->asps;
  m_au.atlas[j].afps = m_atlasAu[j]->afps;
  decodeBlockToPatchMap(j);
  decodePatchParamsList(j);
}

void MivDecoder::decodeBlockToPatchMap(uint8_t j) {
  auto &btpm = m_au.atlas[j].blockToPatchMap;
  const auto &asps = m_au.atlas[j].asps;
  btpm = Common::BlockToPatchMap{
      asps.asps_frame_width() >> asps.asps_log2_patch_packing_block_size(),
      asps.asps_frame_height() >> asps.asps_log2_patch_packing_block_size()};
  std::fill(btpm.getPlane(0).begin(), btpm.getPlane(0).end(), Common::unusedPatchId);

  m_atlasAu[j]->atl.atlas_tile_data_unit().visit([&btpm](size_t p,
                                                         MivBitstream::AtduPatchMode /* unused */,
                                                         const MivBitstream::PatchInformationData
                                                             &pid) {
    const auto &pdu = pid.patch_data_unit();
    const auto first = Common::Vec2i{pdu.pdu_2d_pos_x(), pdu.pdu_2d_pos_y()};
    const auto last = first + Common::Vec2i{pdu.pdu_2d_size_x_minus1(), pdu.pdu_2d_size_y_minus1()};

    for (int y = first.y(); y <= last.y(); ++y) {
      for (int x = first.x(); x <= last.x(); ++x) {
        btpm.getPlane(0)(y, x) = uint16_t(p);
      }
    }
  });
}

void MivDecoder::decodePatchParamsList(uint8_t j) {
  const auto &ath = m_atlasAu[j]->atl.atlas_tile_header();
  VERIFY_MIVBITSTREAM(ath.ath_type() == MivBitstream::AthType::I_TILE ||
                      ath.ath_type() == MivBitstream::AthType::SKIP_TILE);
  if (ath.ath_type() == MivBitstream::AthType::SKIP_TILE) {
    return;
  }

  const auto &atdu = m_atlasAu[j]->atl.atlas_tile_data_unit();
  const auto &asps = m_atlasAu[j]->asps;
  auto &ppl = m_au.atlas[j].patchParamsList;
  ppl.assign(atdu.atduTotalNumberOfPatches(), {});

  atdu.visit([&](size_t p, MivBitstream::AtduPatchMode /* unused */,
                 const MivBitstream::PatchInformationData &pid) {
    const auto &pdu = pid.patch_data_unit();
    const auto k = asps.asps_log2_patch_packing_block_size();

    ppl[p].pduOrientationIndex(pdu.pdu_orientation_index());
    ppl[p].pdu2dPos({int(pdu.pdu_2d_pos_x() << k), int(pdu.pdu_2d_pos_y() << k)});
    ppl[p].pdu2dSize(
        {int((pdu.pdu_2d_size_x_minus1() + 1U) << k), int((pdu.pdu_2d_size_y_minus1() + 1U) << k)});
    ppl[p].pduViewPos({pdu.pdu_view_pos_x(), pdu.pdu_view_pos_y()});
    ppl[p].pduDepthStart(pdu.pdu_depth_start() << ath.ath_pos_min_z_quantizer());
    ppl[p].pduViewIdx(pdu.pdu_view_idx());

    if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
      ppl[p].pduDepthEnd(pdu.pdu_depth_end() << ath.ath_pos_delta_max_z_quantizer());
    }
    if (asps.asps_miv_extension_flag()) {
      ppl[p].pduEntityId(pdu.pdu_miv_extension().pdu_entity_id());

      if (asps.asps_miv_extension().asme_depth_occ_threshold_flag()) {
        ppl[p].pduDepthOccMapThreshold(pdu.pdu_miv_extension().pdu_depth_occ_threshold());
      }
    }
  });
}

auto MivDecoder::decodeOccVideo(uint8_t j) -> bool {
  const double t0 = clock();

  if (m_occVideoDecoder[j]) {
    auto frame = m_occVideoDecoder[j]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[j].decOccFrame = frame->as<Common::YUV400P10>();
    m_occVideoDecoder[j]->wait();
  } else if (m_occFrameServer) {
    m_au.atlas[j].decOccFrame = m_occFrameServer(m_au.vps.vps_atlas_id(j), m_au.foc,
                                                 m_au.atlas[j].decOccFrameSize(m_au.vps));
    if (m_au.atlas[j].decOccFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band occupancy video data but no frame server provided");
  }

  m_totalOccVideoDecodingTime += (clock() - t0) / CLOCKS_PER_SEC;
  return true;
}

auto MivDecoder::decodeGeoVideo(uint8_t j) -> bool {
  const double t0 = clock();

  if (m_geoVideoDecoder[j]) {
    auto frame = m_geoVideoDecoder[j]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[j].decGeoFrame = frame->as<Common::YUV400P10>();
    m_geoVideoDecoder[j]->wait();
  } else if (m_geoFrameServer) {
    m_au.atlas[j].decGeoFrame = m_geoFrameServer(m_au.vps.vps_atlas_id(j), m_au.foc,
                                                 m_au.atlas[j].decGeoFrameSize(m_au.vps));
    if (m_au.atlas[j].decGeoFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band geometry video data but no frame server provided");
  }

  m_totalGeoVideoDecodingTime += (clock() - t0) / CLOCKS_PER_SEC;
  return true;
}

auto MivDecoder::decodeAttrVideo(uint8_t j) -> bool {
  const double t0 = clock();

  if (m_attrVideoDecoder[j]) {
    auto frame = m_attrVideoDecoder[j]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[j].attrFrame = frame->as<Common::YUV444P10>();
    m_attrVideoDecoder[j]->wait();
  } else if (m_attrFrameServer) {
    m_au.atlas[j].attrFrame =
        m_attrFrameServer(m_au.vps.vps_atlas_id(j), m_au.foc, m_au.atlas[j].frameSize());
    if (m_au.atlas[j].attrFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band attribute video data but no frame server provided");
  }

  m_totalAttrVideoDecodingTime += (clock() - t0) / CLOCKS_PER_SEC;
  return true;
}
} // namespace TMIV::Decoder
