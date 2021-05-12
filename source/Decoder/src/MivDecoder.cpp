/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/verify.h>
#include <TMIV/VideoDecoder/VideoDecoderFactory.h>

#include <fmt/format.h>

#include <ctime>
#include <iostream>
#include <utility>

namespace TMIV::Decoder {
MivDecoder::MivDecoder(V3cUnitSource source) : m_inputBuffer{std::move(source)} {}

MivDecoder::~MivDecoder() {
  if (m_totalOccVideoDecodingTime > 0.) {
    fmt::print("Total ocupancy video sub bitstream decoding time: {} s\n",
               m_totalOccVideoDecodingTime);
  }
  if (m_totalGeoVideoDecodingTime > 0.) {
    fmt::print("Total geometry video sub bitstream decoding time: {} s\n",
               m_totalGeoVideoDecodingTime);
  }
  if (m_totalAttrVideoDecodingTime > 0.) {
    fmt::print("Total attribute video sub bitstream decoding time: {} s\n",
               m_totalAttrVideoDecodingTime);
  }
}

void MivDecoder::setOccFrameServer(OccFrameServer value) { m_occFrameServer = std::move(value); }

void MivDecoder::setGeoFrameServer(GeoFrameServer value) { m_geoFrameServer = std::move(value); }

void MivDecoder::setTextureFrameServer(TextureFrameServer value) {
  m_textureFrameServer = std::move(value);
}

void MivDecoder::setTransparencyFrameServer(TransparencyFrameServer value) {
  m_transparencyFrameServer = std::move(value);
}

auto MivDecoder::operator()() -> std::optional<MivBitstream::AccessUnit> {
  m_au.irap = expectIrap();

  if (m_au.irap) {
    if (auto vps = decodeVps()) {
      m_au.vps = *vps;
      resetDecoder();
    } else {
      return std::nullopt;
    }
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

  if (decodeVideoSubBitstreams()) {
    // TODO(BK): This copies the video frames.
    return m_au;
  }
  return {};
}

auto MivDecoder::expectIrap() const -> bool { return !m_commonAtlasDecoder; }

auto MivDecoder::decodeVps() -> std::optional<MivBitstream::V3cParameterSet> {
  if (const auto &vu =
          m_inputBuffer(MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_VPS})) {
    return vu->v3c_unit_payload().v3c_parameter_set();
  }
  return std::nullopt;
}

void MivDecoder::resetDecoder() {
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
  m_textureVideoDecoder.clear();
  m_transparencyVideoDecoder.clear();

  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_au.vps.vps_atlas_id(k);
    auto vuhCad = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_AD};
    vuhCad.vuh_atlas_id(j);
    m_atlasDecoder.push_back(std::make_unique<AtlasDecoder>(
        [this, vuhCad]() { return m_inputBuffer(vuhCad); }, vuhCad, m_au.vps, m_au.foc));
    m_au.atlas.emplace_back();

    if (m_au.vps.vps_occupancy_video_present_flag(j)) {
      auto vuhOvd = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_OVD};
      vuhOvd.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id()).vuh_atlas_id(j);
      m_occVideoDecoder.push_back(startVideoDecoder(vuhOvd, m_totalOccVideoDecodingTime));
    } else {
      m_occVideoDecoder.push_back(nullptr);
    }

    if (m_au.vps.vps_geometry_video_present_flag(j)) {
      auto vuhGvd = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_GVD};
      vuhGvd.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id()).vuh_atlas_id(j);
      m_geoVideoDecoder.push_back(startVideoDecoder(vuhGvd, m_totalGeoVideoDecodingTime));
    } else {
      m_geoVideoDecoder.push_back(nullptr);
    }

    bool attrTextureAbsent = true;
    bool attrTransparencyAbsent = true;
    if (m_au.vps.vps_attribute_video_present_flag(j)) {
      const auto &ai = m_au.vps.attribute_information(j);
      for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
        const auto type = ai.ai_attribute_type_id(i);
        auto vuhAvd = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_AVD};
        vuhAvd.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id()).vuh_atlas_id(j);
        vuhAvd.vuh_attribute_index(i);
        if (type == MivBitstream::AiAttributeTypeId::ATTR_TEXTURE) {
          attrTextureAbsent = false;
          m_textureVideoDecoder.push_back(startVideoDecoder(vuhAvd, m_totalAttrVideoDecodingTime));
        } else if (type == MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY) {
          attrTransparencyAbsent = false;
          m_transparencyVideoDecoder.push_back(
              startVideoDecoder(vuhAvd, m_totalAttrVideoDecodingTime));
        }
      }
    }
    if (attrTextureAbsent) {
      m_textureVideoDecoder.push_back(nullptr);
    }
    if (attrTransparencyAbsent) {
      m_transparencyVideoDecoder.push_back(nullptr);
    }
  }
}

auto MivDecoder::decodeVideoSubBitstreams() -> bool {
  auto result = std::array{false, false};

  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_au.vps.vps_atlas_id(k);

    if (m_au.vps.vps_occupancy_video_present_flag(j)) {
      Common::at(result, static_cast<size_t>(decodeOccVideo(k))) = true;
    }
    if (m_au.vps.vps_geometry_video_present_flag(j)) {
      Common::at(result, static_cast<size_t>(decodeGeoVideo(k))) = true;
    }

    // Note(FT): test the type of attribute to decode : texture AND/OR transparency
    for (uint8_t attributeIndex = 0;
         attributeIndex < m_au.vps.attribute_information(j).ai_attribute_count();
         attributeIndex++) {
      if (m_au.vps.attribute_information(j).ai_attribute_type_id(attributeIndex) ==
          MivBitstream::AiAttributeTypeId::ATTR_TEXTURE) {
        Common::at(result, static_cast<size_t>(decodeAttrTextureVideo(k))) = true;
      }
      if (m_au.vps.attribute_information(j).ai_attribute_type_id(attributeIndex) ==
          MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY) {
        Common::at(result, static_cast<size_t>(decodeAttrTransparencyVideo(k))) = true;
      }
    }
  }

  if (result[0U] && result[1U]) {
    throw std::runtime_error("One of the video streams is truncated");
  }
  return result[1U];
}

void MivDecoder::checkCapabilities() const {
  CONSTRAIN_PTL(m_au.vps.profile_tier_level().ptl_profile_toolset_idc() ==
                    MivBitstream::PtlProfilePccToolsetIdc::MIV_Main ||
                m_au.vps.profile_tier_level().ptl_profile_toolset_idc() ==
                    MivBitstream::PtlProfilePccToolsetIdc::MIV_Extended ||
                m_au.vps.profile_tier_level().ptl_profile_toolset_idc() ==
                    MivBitstream::PtlProfilePccToolsetIdc::MIV_Geometry_Absent);
  CONSTRAIN_PTL(m_au.vps.profile_tier_level().ptl_profile_reconstruction_idc() ==
                MivBitstream::PtlProfileReconstructionIdc::MIV_Main);

  VERIFY_MIVBITSTREAM(m_au.vps.vps_miv_extension_present_flag());
  VERIFY_V3CBITSTREAM(m_au.vps.vps_extension_6bits() == 0);

  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_au.vps.vps_atlas_id(k);
    VERIFY_MIVBITSTREAM(m_au.vps.vps_map_count_minus1(j) == 0);
    VERIFY_MIVBITSTREAM(!m_au.vps.vps_auxiliary_video_present_flag(j));
  }
}

namespace {
auto clockInSeconds() {
  return static_cast<double>(std::clock()) / static_cast<double>(CLOCKS_PER_SEC);
}
} // namespace

auto MivDecoder::startVideoDecoder(const MivBitstream::V3cUnitHeader &vuh, double &totalTime)
    -> std::unique_ptr<VideoDecoder::IVideoDecoder> {
  // Adapt the V3C unit buffer to a video sub-bitstream source
  const auto videoSubBitstreamSource = [this, vuh]() -> std::string {
    if (auto v3cUnit = m_inputBuffer(vuh)) {
      return v3cUnit->v3c_unit_payload().video_sub_bitstream().data();
    }
    return {};
  };

  // Test if the first unit can be read
  auto blob = videoSubBitstreamSource();
  if (blob.empty()) {
    return {}; // The fall-back is to require out-of-band video
  }

  // Adapt the video sub-bitstream source to an ISOBMFF NAL unit source.
  const auto nalUnitSource = [videoSubBitstreamSource,
                              buffer = std::move(blob)]() mutable -> std::string {
    // Pull in data when needed
    if (buffer.empty()) {
      buffer = videoSubBitstreamSource();
    }
    if (buffer.empty()) {
      return {};
    }
    // Decode the NAL unit size
    // NOTE(#494): For V3C, LengthSizeMinusOne is equal to 3.
    std::istringstream stream{buffer.substr(0, 4)};
    const auto size = Common::getUint32(stream);

    // Decode the payload bytes
    auto payload = buffer.substr(4, size);
    VERIFY_V3CBITSTREAM(payload.size() == size);
    buffer = buffer.substr(4 + size);

    // Return the NAL unit of the video sub-bitstream
    return payload;
  };

  const auto t0 = clockInSeconds();
  auto videoDecoder = VideoDecoder::create(
      nalUnitSource, m_au.vps.profile_tier_level().ptl_profile_codec_group_idc());
  totalTime += clockInSeconds() - t0;
  return videoDecoder;
}

void MivDecoder::decodeCommonAtlas() {
  decodeViewParamsList();
  m_au.gup = m_commonAtlasAu->gup;
  m_au.vs = m_commonAtlasAu->vs;
  m_au.vcp = m_commonAtlasAu->vcp;
  m_au.vp = m_commonAtlasAu->vp;
  m_au.casps = m_commonAtlasAu->casps;
}

void MivDecoder::decodeViewParamsList() {
  const auto &caf = m_commonAtlasAu->caf;
  if (caf.caf_extension_present_flag() && caf.caf_miv_extension_present_flag()) {
    const auto &came = caf.caf_miv_extension();
    bool dqParamsPresentFlag = true;
    if (m_commonAtlasAu->casps.casps_extension_present_flag() &&
        m_commonAtlasAu->casps.casps_miv_extension_present_flag()) {
      dqParamsPresentFlag = m_commonAtlasAu->casps.casps_miv_extension()
                                .casme_depth_quantization_params_present_flag();
    }
    if (m_commonAtlasAu->irap) {
      decodeMvpl(came.miv_view_params_list(), dqParamsPresentFlag);
    } else {
      if (came.came_update_extrinsics_flag()) {
        decodeMvpue(came.miv_view_params_update_extrinsics());
      }
      if (came.came_update_intrinsics_flag()) {
        decodeMvpui(came.miv_view_params_update_intrinsics());
      }
      if (m_commonAtlasAu->casps.casps_miv_extension()
              .casme_depth_quantization_params_present_flag() &&
          came.came_update_depth_quantization_flag() && dqParamsPresentFlag) {
        decodeMvpudq(came.miv_view_params_update_depth_quantization());
      }
    }
  }

  if (m_commonAtlasAu->casps.casps_extension_present_flag() &&
      m_commonAtlasAu->casps.casps_miv_extension_present_flag()) {
    const auto &casme = m_commonAtlasAu->casps.casps_miv_extension();
    if (casme.casme_vui_params_present_flag()) {
      const auto &vui = casme.vui_parameters();
      VERIFY_MIVBITSTREAM(!m_au.vui || *m_au.vui == vui);
      m_au.vui = vui;
    }
  }
}

void MivDecoder::decodeMvpl(const MivBitstream::MivViewParamsList &mvpl, bool dqParamsPresentFlag) {
  m_au.viewParamsList.assign(mvpl.mvp_num_views_minus1() + size_t{1}, {});

  for (uint16_t viewIdx = 0; viewIdx <= mvpl.mvp_num_views_minus1(); ++viewIdx) {
    auto &vp = m_au.viewParamsList[viewIdx];
    vp.pose = MivBitstream::Pose::decodeFrom(mvpl.camera_extrinsics(viewIdx));
    vp.isInpainted = mvpl.mvp_inpaint_flag(viewIdx);
    vp.ci = mvpl.camera_intrinsics(viewIdx);
    if (dqParamsPresentFlag) {
      vp.dq = mvpl.depth_quantization(viewIdx);
    }
    if (mvpl.mvp_pruning_graph_params_present_flag()) {
      const auto viewId = mvpl.viewIndexToId(viewIdx);
      vp.pp = mvpl.pruning_parent(viewId);
    }

    vp.name = fmt::format("pv{:02}", viewIdx);
  }
}

void MivDecoder::decodeMvpue(const MivBitstream::MivViewParamsUpdateExtrinsics &mvpue) {
  for (uint16_t i = 0; i <= mvpue.mvpue_num_view_updates_minus1(); ++i) {
    m_au.viewParamsList[mvpue.mvpue_view_idx(i)].pose =
        MivBitstream::Pose::decodeFrom(mvpue.camera_extrinsics(i));
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
  const auto &ppl = decodePatchParamsList(k, m_au.atlas[k].patchParamsList);
  requireAllPatchesWithinProjectionPlaneBounds(m_au.viewParamsList, ppl);
  m_au.atlas[k].blockToPatchMap = decodeBlockToPatchMap(k, ppl);
}

// NOTE(BK): Combined implementation of two processes because there is only a single tile in MIV
// main profile:
//  * [WG 07 N 0003:9.2.6]   Decoding process of the block to patch map
//  * [WG 07 N 0003:9.2.7.2] Conversion of tile level blockToPatch information to atlas level
//                           blockToPatch information
auto MivDecoder::decodeBlockToPatchMap(size_t k, const MivBitstream::PatchParamsList &ppl) const
    -> Common::BlockToPatchMap {
  const auto &asps = m_au.atlas[k].asps;

  const std::int32_t log2PatchPackingBlockSize = asps.asps_log2_patch_packing_block_size();
  const auto patchPackingBlockSize = 1 << log2PatchPackingBlockSize;
  const auto offset = patchPackingBlockSize - 1;

  const auto atlasBlockToPatchMapWidth = (asps.asps_frame_width() + offset) / patchPackingBlockSize;
  const auto atlasBlockToPatchMapHeight =
      (asps.asps_frame_height() + offset) / patchPackingBlockSize;

  // All elements of TileBlockToPatchMap are first initialized to -1 as follows [9.2.6]
  auto btpm = Common::BlockToPatchMap{atlasBlockToPatchMapWidth, atlasBlockToPatchMapHeight};
  std::fill(btpm.getPlane(0).begin(), btpm.getPlane(0).end(), Common::unusedPatchId);

  // Then the AtlasBlockToPatchMap array is updated as follows:
  for (size_t p = 0; p < ppl.size(); ++p) {
    const size_t xOrg = ppl[p].atlasPatch2dPosX() / patchPackingBlockSize;
    const size_t yOrg = ppl[p].atlasPatch2dPosY() / patchPackingBlockSize;
    const size_t atlasPatchWidthBlk = (ppl[p].atlasPatch2dSizeX() + offset) / patchPackingBlockSize;
    const size_t atlasPatchHeightBlk =
        (ppl[p].atlasPatch2dSizeY() + offset) / patchPackingBlockSize;

    for (size_t y = 0; y < atlasPatchHeightBlk; ++y) {
      for (size_t x = 0; x < atlasPatchWidthBlk; ++x) {
        if (!asps.asps_patch_precedence_order_flag() ||
            btpm.getPlane(0)(yOrg + y, xOrg + x) == Common::unusedPatchId) {
          btpm.getPlane(0)(yOrg + y, xOrg + x) = static_cast<uint16_t>(p);
        }
      }
    }
  }

  return btpm;
}

auto MivDecoder::decodePatchParamsList(size_t k, MivBitstream::PatchParamsList &ppl) const
    -> const MivBitstream::PatchParamsList & {
  const auto &ath = m_atlasAu[k]->atl.atlas_tile_header();
  VERIFY_MIVBITSTREAM(ath.ath_type() == MivBitstream::AthType::I_TILE ||
                      ath.ath_type() == MivBitstream::AthType::SKIP_TILE);
  if (ath.ath_type() == MivBitstream::AthType::SKIP_TILE) {
    return ppl;
  }

  const auto &atdu = m_atlasAu[k]->atl.atlas_tile_data_unit();
  const auto &asps = m_atlasAu[k]->asps;
  const auto &afps = m_atlasAu[k]->afps;

  ppl.assign(atdu.atduTotalNumberOfPatches(), {});
  atdu.visit([&](size_t p, MivBitstream::AtduPatchMode /* unused */,
                 const MivBitstream::PatchInformationData &pid) {
    ppl[p] = MivBitstream::PatchParams::decodePdu(pid.patch_data_unit(), asps, afps, ath);
  });

  return ppl;
}

// TODO(m56532): Generalize to a decoding _any_ video sub-bitstream in a consistently
auto MivDecoder::decodeOccVideo(size_t k) -> bool {
  fmt::print("Decode frame V3C_OVD {} FOC={}\n", m_au.vps.vps_atlas_id(k), m_au.foc);
  const auto t0 = clockInSeconds();

  if (m_occVideoDecoder[k]) {
    auto frame = m_occVideoDecoder[k]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[k].decOccFrame = frame->as<Common::YUV400P10>();
  } else if (m_occFrameServer) {
    m_au.atlas[k].decOccFrame = m_occFrameServer(m_au.vps.vps_atlas_id(k), m_au.foc,
                                                 m_au.atlas[k].decOccFrameSize(m_au.vps));
    if (m_au.atlas[k].decOccFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band occupancy video data but no frame server provided");
  }

  m_totalOccVideoDecodingTime += clockInSeconds() - t0;
  return true;
}

auto MivDecoder::decodeGeoVideo(size_t k) -> bool {
  fmt::print("Decode frame V3C_GVD {} FOC={}\n", m_au.vps.vps_atlas_id(k), m_au.foc);
  const auto t0 = clockInSeconds();

  if (m_geoVideoDecoder[k]) {
    auto frame = m_geoVideoDecoder[k]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[k].decGeoFrame = frame->as<Common::YUV400P10>();
  } else if (m_geoFrameServer) {
    m_au.atlas[k].decGeoFrame = m_geoFrameServer(m_au.vps.vps_atlas_id(k), m_au.foc,
                                                 m_au.atlas[k].decGeoFrameSize(m_au.vps));
    if (m_au.atlas[k].decGeoFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band geometry video data but no frame server provided");
  }

  m_totalGeoVideoDecodingTime += clockInSeconds() - t0;
  return true;
}

auto MivDecoder::decodeAttrTextureVideo(size_t k) -> bool {
  fmt::print("Decode frame V3C_AVD {} {} FOC={}\n", m_au.vps.vps_atlas_id(k),
             MivBitstream::AiAttributeTypeId::ATTR_TEXTURE, m_au.foc);
  const auto t0 = clockInSeconds();

  if (m_textureVideoDecoder[k]) {
    auto frame = m_textureVideoDecoder[k]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[k].attrFrame = frame->as<Common::YUV444P10>();
  } else if (m_textureFrameServer) {
    m_au.atlas[k].attrFrame =
        m_textureFrameServer(m_au.vps.vps_atlas_id(k), m_au.foc, m_au.atlas[k].frameSize());
    if (m_au.atlas[k].attrFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band texture video data but no frame server provided");
  }

  m_totalAttrVideoDecodingTime += clockInSeconds() - t0;
  return true;
}

auto MivDecoder::decodeAttrTransparencyVideo(size_t k) -> bool {
  fmt::print("Decode frame V3C_AVD {} {} FOC={}\n", m_au.vps.vps_atlas_id(k),
             MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY, m_au.foc);
  const auto t0 = clockInSeconds();

  if (m_transparencyVideoDecoder[k]) {
    auto frame = m_transparencyVideoDecoder[k]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[k].transparencyFrame = frame->as<Common::YUV400P10>();
  } else if (m_transparencyFrameServer) {
    m_au.atlas[k].transparencyFrame =
        m_transparencyFrameServer(m_au.vps.vps_atlas_id(k), m_au.foc, m_au.atlas[k].frameSize());
    if (m_au.atlas[k].transparencyFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band transparency video data but no frame server provided");
  }

  m_totalAttrVideoDecodingTime += clockInSeconds() - t0;
  return true;
}

void MivDecoder::summarizeVps() const {
  const auto &vps = m_au.vps;
  const auto &ptl = vps.profile_tier_level();

  fmt::print("V3C parameter set {}:\n", vps.vps_v3c_parameter_set_id());
  fmt::print("  Tier {}, {}, codec group {}, toolset {}, recon {}, decodes {}\n",
             ptl.ptl_tier_flag(), ptl.ptl_level_idc(), ptl.ptl_profile_codec_group_idc(),
             ptl.ptl_profile_toolset_idc(), ptl.ptl_profile_reconstruction_idc(),
             ptl.ptl_max_decodes_idc());
  for (size_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
    const auto j = vps.vps_atlas_id(k);
    fmt::print("  Atlas {}: {} x {}", j, vps.vps_frame_width(j), vps.vps_frame_height(j));
    if (vps.vps_occupancy_video_present_flag(j)) {
      const auto &oi = vps.occupancy_information(j);
      fmt::print("; [OI: codec {}, {}, 2D {}, align {}]", oi.oi_occupancy_codec_id(),
                 oi.oi_lossy_occupancy_compression_threshold(),
                 oi.oi_occupancy_2d_bit_depth_minus1() + 1, oi.oi_occupancy_MSB_align_flag());
    }
    if (vps.vps_geometry_video_present_flag(j)) {
      const auto &gi = vps.geometry_information(j);
      fmt::print("; [GI: codec {}, 2D {}, algin {}, 3D {}]", gi.gi_geometry_codec_id(),
                 gi.gi_geometry_2d_bit_depth_minus1() + 1, gi.gi_geometry_MSB_align_flag(),
                 gi.gi_geometry_3d_coordinates_bit_depth_minus1() + 1);
    }
    if (vps.vps_attribute_video_present_flag(j)) {
      const auto &ai = vps.attribute_information(j);
      fmt::print("; [AI: {}", ai.ai_attribute_count());
      for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
        fmt::print(", {}, codec {}, dims {}, 2D {}, align {}", ai.ai_attribute_type_id(i),
                   ai.ai_attribute_codec_id(i), ai.ai_attribute_dimension_minus1(i) + 1,
                   ai.ai_attribute_2d_bit_depth_minus1(i) + 1, ai.ai_attribute_MSB_align_flag(i));
        if (i + 1 == ai.ai_attribute_count()) {
          fmt::print("]");
        }
      }
    }
    fmt::print("\n");
  }
  const auto &vme = vps.vps_miv_extension();
  fmt::print(", geometry scaling {}, groups {}, embedded occupancy {}, occupancy scaling {}\n",
             vme.vme_geometry_scale_enabled_flag(), vme.group_mapping().gm_group_count(),
             vme.vme_embedded_occupancy_enabled_flag(), vme.vme_occupancy_scale_enabled_flag());
}
} // namespace TMIV::Decoder
