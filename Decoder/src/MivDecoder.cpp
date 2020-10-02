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

void MivDecoder::setOccFrameServer(OccFrameServer value) { m_occFrameServer = std::move(value); }

void MivDecoder::setGeoFrameServer(GeoFrameServer value) { m_geoFrameServer = std::move(value); }

void MivDecoder::setAttrFrameServer(AttrFrameServer value) { m_attrFrameServer = std::move(value); }

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

  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    if (!m_atlasAu[k] || m_atlasAu[k]->foc < m_au.foc) {
      m_atlasAu[k] = (*m_atlasDecoder[k])();
    }
    if (m_atlasAu[k] && m_atlasAu[k]->foc == m_au.foc) {
      decodeAtlas(k);
    }
  }

  auto result = std::array{false, false};

  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_au.vps.vps_atlas_id(k);
    if (m_au.vps.vps_occupancy_video_present_flag(j)) {
      result[static_cast<std::size_t>(decodeOccVideo(k))] = true;
    }
  }
  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_au.vps.vps_atlas_id(k);
    if (m_au.vps.vps_geometry_video_present_flag(j)) {
      result[static_cast<std::size_t>(decodeGeoVideo(k))] = true;
    }
  }
  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_au.vps.vps_atlas_id(k);
    if (m_au.vps.vps_attribute_video_present_flag(j)) {
      result[static_cast<std::size_t>(decodeAttrVideo(k))] = true;
    }
  }

  if (result[0U] && result[1U]) {
    throw std::runtime_error("One of the video streams is truncated");
  }
  if (result[1U]) {
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
  m_au.vps = vu->v3c_unit_payload().v3c_parameter_set();

  summarizeVps();
  checkCapabilities();

  auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_CAD};
  m_commonAtlasDecoder = std::make_unique<CommonAtlasDecoder>(
      [this, vuh]() { return m_inputBuffer(vuh); }, m_au.vps, m_au.foc);

  m_atlasDecoder.clear();
  m_atlasAu.assign(m_au.vps.vps_atlas_count_minus1() + size_t{1}, {});
  m_au.atlas.clear();
  m_occVideoDecoder.clear();
  m_geoVideoDecoder.clear();
  m_attrVideoDecoder.clear();

  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_au.vps.vps_atlas_id(k);
    auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_AD};
    vuh.vuh_atlas_id(j);
    m_atlasDecoder.push_back(std::make_unique<AtlasDecoder>(
        [this, vuh]() { return m_inputBuffer(vuh); }, vuh, m_au.vps, m_au.foc));
    m_au.atlas.emplace_back();

    if (m_au.vps.vps_occupancy_video_present_flag(j)) {
      auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_OVD};
      vuh.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id()).vuh_atlas_id(j);
      m_occVideoDecoder.push_back(startVideoDecoder(vuh, m_totalOccVideoDecodingTime));
    } else {
      m_occVideoDecoder.push_back(nullptr);
    }

    if (m_au.vps.vps_geometry_video_present_flag(j)) {
      auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_GVD};
      vuh.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id()).vuh_atlas_id(j);
      m_geoVideoDecoder.push_back(startVideoDecoder(vuh, m_totalGeoVideoDecodingTime));
    }

    if (m_au.vps.vps_attribute_video_present_flag(j)) {
      auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_AVD};
      vuh.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id()).vuh_atlas_id(j);
      m_attrVideoDecoder.push_back(startVideoDecoder(vuh, m_totalAttrVideoDecodingTime));
    }
  }

  return true;
}

void MivDecoder::checkCapabilities() const {
  CONSTRAIN_PTL(m_au.vps.profile_tier_level().ptl_profile_codec_group_idc() ==
                MivBitstream::PtlProfileCodecGroupIdc::HEVC_Main10);
  CONSTRAIN_PTL(m_au.vps.profile_tier_level().ptl_profile_toolset_idc() ==
                MivBitstream::PtlProfilePccToolsetIdc::MIV_Main);
  CONSTRAIN_PTL(m_au.vps.profile_tier_level().ptl_profile_reconstruction_idc() ==
                MivBitstream::PtlProfileReconstructionIdc::MIV_Main);

  VERIFY_MIVBITSTREAM(m_au.vps.vps_miv_extension_present_flag());
  VERIFY_V3CBITSTREAM(m_au.vps.vps_extension_7bits() == 0);

  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_au.vps.vps_atlas_id(k);
    VERIFY_MIVBITSTREAM(m_au.vps.vps_map_count_minus1(j) == 0);
    VERIFY_MIVBITSTREAM(!m_au.vps.vps_auxiliary_video_present_flag(j));
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
    data += vu->v3c_unit_payload().video_sub_bitstream().data();
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
  m_au.gup = m_commonAtlasAu->gup;
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

  if (m_commonAtlasAu->aaps.aaps_miv_extension_present_flag()) {
    const auto &aame = m_commonAtlasAu->aaps.aaps_miv_extension();
    if (aame.aame_vui_params_present_flag()) {
      const auto &vui = aame.vui_parameters();
      VERIFY_MIVBITSTREAM(!m_au.vui || *m_au.vui == vui);
      m_au.vui = vui;
    }
  }
}

void MivDecoder::decodeMvpl(const MivBitstream::MivViewParamsList &mvpl) {
  m_au.viewParamsList.assign(mvpl.mvp_num_views_minus1() + size_t{1}, {});

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

void MivDecoder::decodeAtlas(size_t k) {
  m_au.atlas[k].asps = m_atlasAu[k]->asps;
  m_au.atlas[k].afps = m_atlasAu[k]->afps;
  decodeBlockToPatchMap(k);
  decodePatchParamsList(k);
}

void MivDecoder::decodeBlockToPatchMap(size_t k) {
  auto &btpm = m_au.atlas[k].blockToPatchMap;
  const auto &asps = m_au.atlas[k].asps;
  btpm = Common::BlockToPatchMap{
      asps.asps_frame_width() >> asps.asps_log2_patch_packing_block_size(),
      asps.asps_frame_height() >> asps.asps_log2_patch_packing_block_size()};
  std::fill(btpm.getPlane(0).begin(), btpm.getPlane(0).end(), Common::unusedPatchId);

  m_atlasAu[k]->atl.atlas_tile_data_unit().visit(
      [&btpm](size_t p, MivBitstream::AtduPatchMode /* unused */,
              const MivBitstream::PatchInformationData &pid) {
        const auto &pdu = pid.patch_data_unit();
        const auto firstX = pdu.pdu_2d_pos_x();
        const auto firstY = pdu.pdu_2d_pos_y();
        const auto lastX = firstX + pdu.pdu_2d_size_x_minus1();
        const auto lastY = firstY + pdu.pdu_2d_size_y_minus1();

        for (auto y = firstY; y <= lastY; ++y) {
          for (auto x = firstX; x <= lastX; ++x) {
            btpm.getPlane(0)(y, x) = static_cast<uint16_t>(p);
          }
        }
      });
}

void MivDecoder::decodePatchParamsList(size_t k) {
  const auto &ath = m_atlasAu[k]->atl.atlas_tile_header();
  VERIFY_MIVBITSTREAM(ath.ath_type() == MivBitstream::AthType::I_TILE ||
                      ath.ath_type() == MivBitstream::AthType::SKIP_TILE);
  if (ath.ath_type() == MivBitstream::AthType::SKIP_TILE) {
    return;
  }

  const auto &atdu = m_atlasAu[k]->atl.atlas_tile_data_unit();
  const auto &asps = m_atlasAu[k]->asps;
  auto &ppl = m_au.atlas[k].patchParamsList;
  ppl.assign(atdu.atduTotalNumberOfPatches(), {});

  const auto patchPackingBlockSize = 1U << asps.asps_log2_patch_packing_block_size();
  const auto offsetDQuantizer = 1U << ath.ath_pos_min_d_quantizer();
  const auto rangeDQuantizer = 1U << ath.ath_pos_delta_max_d_quantizer();
  const auto rangeDBitDepth = std::min(asps.asps_geometry_2d_bit_depth_minus1() + 1U,
                                       asps.asps_geometry_3d_bit_depth_minus1() + 1U);
  const auto rangeD = 1U << rangeDBitDepth;
  const auto patchSizeXQuantizer = asps.asps_patch_size_quantizer_present_flag()
                                       ? 1U << ath.ath_patch_size_x_info_quantizer()
                                       : patchPackingBlockSize;
  const auto patchSizeYQuantizer = asps.asps_patch_size_quantizer_present_flag()
                                       ? 1U << ath.ath_patch_size_y_info_quantizer()
                                       : patchPackingBlockSize;

  atdu.visit([&](size_t p, MivBitstream::AtduPatchMode /* unused */,
                 const MivBitstream::PatchInformationData &pid) {
    const auto &pdu = pid.patch_data_unit();

    ppl[p].atlasPatch2dPosX(pdu.pdu_2d_pos_x() * patchPackingBlockSize);
    ppl[p].atlasPatch2dPosY(pdu.pdu_2d_pos_y() * patchPackingBlockSize);
    ppl[p].atlasPatch3dOffsetU(pdu.pdu_3d_offset_u());
    ppl[p].atlasPatch3dOffsetV(pdu.pdu_3d_offset_v());
    ppl[p].atlasPatch3dOffsetD(pdu.pdu_3d_offset_d() * offsetDQuantizer);

    if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
      ppl[p].atlasPatch3dRangeD(
          pdu.pdu_3d_range_d() == 0 ? 0 : (pdu.pdu_3d_range_d() * rangeDQuantizer) - 1);
    } else {
      ppl[p].atlasPatch3dRangeD(rangeD - 1);
    }

    ppl[p].atlasPatchProjectionId(pdu.pdu_projection_id());
    ppl[p].atlasPatchOrientationIndex(pdu.pdu_orientation_index());
    ppl[p].atlasPatch2dSizeX((pdu.pdu_2d_size_x_minus1() + 1) * patchSizeXQuantizer);
    ppl[p].atlasPatch2dSizeY((pdu.pdu_2d_size_y_minus1() + 1) * patchSizeYQuantizer);

    if (asps.asps_miv_extension_present_flag()) {
      ppl[p].atlasPatchEntityId(pdu.pdu_miv_extension().pdu_entity_id());

      if (asps.asps_miv_extension().asme_depth_occ_threshold_flag()) {
        ppl[p].atlasPatchDepthOccMapThreshold(pdu.pdu_miv_extension().pdu_depth_occ_threshold());
      }
    }
  });
}

auto MivDecoder::decodeOccVideo(size_t k) -> bool {
  const double t0 = clock();

  if (m_occVideoDecoder[k]) {
    auto frame = m_occVideoDecoder[k]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[k].decOccFrame = frame->as<Common::YUV400P10>();
    m_occVideoDecoder[k]->wait();
  } else if (m_occFrameServer) {
    m_au.atlas[k].decOccFrame = m_occFrameServer(m_au.vps.vps_atlas_id(k), m_au.foc,
                                                 m_au.atlas[k].decOccFrameSize(m_au.vps));
    if (m_au.atlas[k].decOccFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band occupancy video data but no frame server provided");
  }

  m_totalOccVideoDecodingTime += (clock() - t0) / CLOCKS_PER_SEC;
  return true;
}

auto MivDecoder::decodeGeoVideo(size_t k) -> bool {
  const double t0 = clock();

  if (m_geoVideoDecoder[k]) {
    auto frame = m_geoVideoDecoder[k]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[k].decGeoFrame = frame->as<Common::YUV400P10>();
    m_geoVideoDecoder[k]->wait();
  } else if (m_geoFrameServer) {
    m_au.atlas[k].decGeoFrame = m_geoFrameServer(m_au.vps.vps_atlas_id(k), m_au.foc,
                                                 m_au.atlas[k].decGeoFrameSize(m_au.vps));
    if (m_au.atlas[k].decGeoFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band geometry video data but no frame server provided");
  }

  m_totalGeoVideoDecodingTime += (clock() - t0) / CLOCKS_PER_SEC;
  return true;
}

auto MivDecoder::decodeAttrVideo(size_t k) -> bool {
  const double t0 = clock();

  if (m_attrVideoDecoder[k]) {
    auto frame = m_attrVideoDecoder[k]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[k].attrFrame = frame->as<Common::YUV444P10>();
    m_attrVideoDecoder[k]->wait();
  } else if (m_attrFrameServer) {
    m_au.atlas[k].attrFrame =
        m_attrFrameServer(m_au.vps.vps_atlas_id(k), m_au.foc, m_au.atlas[k].frameSize());
    if (m_au.atlas[k].attrFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band attribute video data but no frame server provided");
  }

  m_totalAttrVideoDecodingTime += (clock() - t0) / CLOCKS_PER_SEC;
  return true;
}

void MivDecoder::summarizeVps() const {
  const auto &vps = m_au.vps;
  const auto &ptl = vps.profile_tier_level();

  std::cout << "V3C parameter set " << int{vps.vps_v3c_parameter_set_id()} << ":\n";
  std::cout << "  Tier " << static_cast<int>(ptl.ptl_tier_flag()) << ", " << ptl.ptl_level_idc()
            << ", codec group " << ptl.ptl_profile_codec_group_idc() << ", toolset "
            << ptl.ptl_profile_toolset_idc() << ", recon " << ptl.ptl_profile_reconstruction_idc()
            << ", decodes " << ptl.ptl_max_decodes_idc() << '\n';
  for (size_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
    const auto j = vps.vps_atlas_id(k);
    std::cout << "  Atlas " << j << ": " << vps.vps_frame_width(j) << " x "
              << vps.vps_frame_height(j);
    if (vps.vps_occupancy_video_present_flag(j)) {
      const auto &oi = vps.occupancy_information(j);
      std::cout << "; [OI: codec " << int{oi.oi_occupancy_codec_id()} << ", "
                << int{oi.oi_lossy_occupancy_compression_threshold()} << ", 2D "
                << (oi.oi_occupancy_2d_bit_depth_minus1() + 1) << ", align " << std::boolalpha
                << oi.oi_occupancy_MSB_align_flag() << ']';
    }
    if (vps.vps_geometry_video_present_flag(j)) {
      const auto &gi = vps.geometry_information(j);
      std::cout << "; [GI: codec " << int{gi.gi_geometry_codec_id()} << ", 2D "
                << (gi.gi_geometry_2d_bit_depth_minus1() + 1) << ", align " << std::boolalpha
                << gi.gi_geometry_MSB_align_flag() << ", 3D "
                << (gi.gi_geometry_3d_coordinates_bit_depth_minus1() + 1) << ']';
    }
    if (vps.vps_attribute_video_present_flag(j)) {
      const auto &ai = vps.attribute_information(j);
      std::cout << "; [AI: " << int{ai.ai_attribute_count()};
      for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
        std::cout << ", " << ai.ai_attribute_type_id(i) << ", codec "
                  << int{ai.ai_attribute_codec_id(i)} << ", dims "
                  << (ai.ai_attribute_dimension_minus1(i) + 1) << ", 2D "
                  << (ai.ai_attribute_2d_bit_depth_minus1(i) + 1) << ", align " << std::boolalpha
                  << ai.ai_attribute_MSB_align_flag(i) << ']';
      }
    }
    std::cout << '\n';
  }
  const auto &vme = vps.vps_miv_extension();
  std::cout << "  MIV: depth low quality " << std::boolalpha << vme.vme_depth_low_quality_flag()
            << ", geometry scaling " << std::boolalpha << vme.vme_geometry_scale_enabled_flag()
            << ", groups " << (vme.vme_num_groups_minus1() + 1) << ", entities "
            << (vme.vme_max_entities_minus1() + 1) << ", embedded occupancy " << std::boolalpha
            << vme.vme_embedded_occupancy_flag() << ", occupancy scaling "
            << vme.vme_occupancy_scale_enabled_flag() << '\n';
}
} // namespace TMIV::Decoder
