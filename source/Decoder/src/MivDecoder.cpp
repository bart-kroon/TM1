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

#include <TMIV/Decoder/MivDecoder.h>

#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/verify.h>
#include <TMIV/PtlChecker/PtlChecker.h>
#include <TMIV/VideoDecoder/VideoDecoderFactory.h>

#include <fmt/format.h>

#include <ctime>
#include <iostream>
#include <utility>

namespace TMIV::Decoder {
MivDecoder::MivDecoder(V3cUnitSource source)
    : m_inputBuffer{std::move(source)}, m_checker{std::make_shared<PtlChecker::PtlChecker>()} {}

MivDecoder::~MivDecoder() {
  for (const auto [vuh, totalTime] : m_totalVideoDecodingTime) {
    if (0. < totalTime) {
      fmt::print("Total {} decoding time: {} s\n", vuh.summary(), totalTime);
    }
  }
}

void MivDecoder::setFrameServer(FrameServer value) { m_frameServer = std::move(value); }

void MivDecoder::replaceChecker(SharedChecker value) {
  PRECONDITION(value);
  m_checker = std::move(value);
}

auto MivDecoder::operator()() -> std::optional<MivBitstream::AccessUnit> {
  VERIFY(m_state != State::eof);
  m_au.irap = !m_commonAtlasDecoder;

  if (m_au.irap) {
    if (auto vps = decodeVps()) {
      m_au.vps = *vps;
      m_checker->checkAndActivateVps(m_au.vps);
      resetDecoder();
    } else if (m_state == State::decoding) {
      m_state = State::eof;
      return std::nullopt;
    } else {
      RUNTIME_ERROR("No VPS in V3C sample stream");
    }
  } else {
    VERIFY(m_state == State::decoding);
  }

  ++m_au.foc;

  if (m_state == State::initial || (m_commonAtlasAu && m_commonAtlasAu->foc < m_au.foc)) {
    m_commonAtlasAu = (*m_commonAtlasDecoder)();
  }
  if (m_commonAtlasAu && m_commonAtlasAu->foc == m_au.foc) {
    decodeCommonAtlas();
  }

  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    if (m_state == State::initial || (m_atlasAu[k] && m_atlasAu[k]->foc < m_au.foc)) {
      m_atlasAu[k] = (*m_atlasDecoder[k])();
    }
    if (m_atlasAu[k] && m_atlasAu[k]->foc == m_au.foc) {
      decodeAtlas(k);
    }
  }

  if (decodeVideoSubBitstreams()) {
    m_state = State::decoding;
    return m_au;
  }
  m_state = State::eof;
  return {};
}

auto MivDecoder::decodeVps() -> std::optional<MivBitstream::V3cParameterSet> {
  if (const auto &vu = m_inputBuffer(MivBitstream::V3cUnitHeader::vps())) {
    m_checker->checkVuh(vu->v3c_unit_header());
    return vu->v3c_unit_payload().v3c_parameter_set();
  }
  return std::nullopt;
}

void MivDecoder::resetDecoder() {
  std::cout << m_au.vps.summary();
  checkCapabilities();

  const auto vpsId = m_au.vps.vps_v3c_parameter_set_id();

  m_commonAtlasDecoder = std::make_unique<CommonAtlasDecoder>(
      [this, vuh = MivBitstream::V3cUnitHeader::cad(vpsId)]() { return m_inputBuffer(vuh); },
      m_au.vps, m_au.foc, m_checker);

  m_atlasDecoder.clear();
  m_atlasAu.assign(m_au.vps.vps_atlas_count_minus1() + size_t{1}, {});
  m_au.atlas.assign(m_au.vps.vps_atlas_count_minus1() + size_t{1}, {});

  for (auto &[vuh, decoder] : m_videoDecoders) {
    decoder.reset();
  }

  for (uint8_t atlasIdx = 0; atlasIdx <= m_au.vps.vps_atlas_count_minus1(); ++atlasIdx) {
    const auto atlasId = m_au.vps.vps_atlas_id(atlasIdx);

    const auto vuhAd = MivBitstream::V3cUnitHeader::ad(vpsId, atlasId);
    m_atlasDecoder.push_back(std::make_unique<AtlasDecoder>(
        [this, vuhAd]() { return m_inputBuffer(vuhAd); }, vuhAd, m_au.vps, m_au.foc, m_checker));

    if (m_au.vps.vps_occupancy_video_present_flag(atlasId)) {
      tryStartVideoDecoder(MivBitstream::V3cUnitHeader::ovd(vpsId, atlasId));
    }

    if (m_au.vps.vps_geometry_video_present_flag(atlasId)) {
      tryStartVideoDecoder(MivBitstream::V3cUnitHeader::gvd(vpsId, atlasId));
    }

    if (m_au.vps.vps_attribute_video_present_flag(atlasId)) {
      const auto &ai = m_au.vps.attribute_information(atlasId);
      m_au.atlas[atlasIdx].decAttrFrame.resize(ai.ai_attribute_count());

      for (uint8_t attrIdx = 0; attrIdx < ai.ai_attribute_count(); ++attrIdx) {
        tryStartVideoDecoder(MivBitstream::V3cUnitHeader::avd(vpsId, atlasId, attrIdx));
      }
    }

    if (m_au.vps.vps_packed_video_present_flag(atlasId)) {
      tryStartVideoDecoder(MivBitstream::V3cUnitHeader::pvd(vpsId, atlasId));
    }
  }
}

auto MivDecoder::decodeVideoSubBitstreams() -> bool {
  for (auto &atlas : m_au.atlas) {
    atlas.decOccFrame.clear();
    atlas.decGeoFrame.clear();

    for (auto &attr : atlas.decAttrFrame) {
      attr.clear();
    }

    atlas.decPckFrame.clear();
  }

  return std::all_of(m_videoDecoders.begin(), m_videoDecoders.end(),
                     [this](auto &kvp) { return decodeVideoFrame(kvp.key); });
}

void MivDecoder::checkCapabilities() const {
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

auto MivDecoder::decoderId(MivBitstream::V3cUnitHeader /* vuh */) const noexcept
    -> VideoDecoder::DecoderId {
  switch (m_au.vps.profile_tier_level().ptl_profile_codec_group_idc()) {
  case MivBitstream::PtlProfileCodecGroupIdc::AVC_Progressive_High:
    return VideoDecoder::DecoderId::AVC_Progressive_High;
  case MivBitstream::PtlProfileCodecGroupIdc::HEVC_Main10:
    return VideoDecoder::DecoderId::HEVC_Main10;
  case MivBitstream::PtlProfileCodecGroupIdc::HEVC444:
    return VideoDecoder::DecoderId::HEVC444;
  case MivBitstream::PtlProfileCodecGroupIdc::VVC_Main10:
    return VideoDecoder::DecoderId::VVC_Main10;
  case MivBitstream::PtlProfileCodecGroupIdc::MP4RA:
    NOT_IMPLEMENTED;
  default:
    UNREACHABLE;
  }
}

auto MivDecoder::tryStartVideoDecoder(MivBitstream::V3cUnitHeader vuh) -> bool {
  auto &decoder = m_videoDecoders[vuh];
  auto &totalTime = m_totalVideoDecodingTime[vuh];

  // Adapt the V3C unit buffer to a video sub-bitstream source
  const auto videoSubBitstreamSource = [this, vuh]() -> std::string {
    if (auto v3cUnit = m_inputBuffer(vuh)) {
      m_checker->checkVuh(vuh);
      return v3cUnit->v3c_unit_payload().video_sub_bitstream().data();
    }
    return {};
  };

  // Test if the first unit can be read
  auto blob = videoSubBitstreamSource();
  if (blob.empty()) {
    return false; // The fall-back is to require out-of-band video
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
    //
    // NOTE(#494): For V3C, LengthSizeMinusOne is equal to 3.
    std::istringstream stream{buffer.substr(0, 4)};
    const auto size = Common::getUint32(stream);

    // Decode the payload bytes
    auto payload = buffer.substr(4, size);
    VERIFY_V3CBITSTREAM(payload.size() == size);
    buffer = buffer.substr(size_t{4} + size);

    // Return the NAL unit of the video sub-bitstream
    return payload;
  };

  const auto t0 = clockInSeconds();

  decoder = VideoDecoder::create(nalUnitSource, decoderId(vuh));

  totalTime += clockInSeconds() - t0;
  return true;
}

namespace {
auto decFrame(MivBitstream::V3cUnitHeader vuh, MivBitstream::AtlasAccessUnit &aau)
    -> Common::Frame<> & {
  switch (vuh.vuh_unit_type()) {
  case MivBitstream::VuhUnitType::V3C_OVD:
    return aau.decOccFrame;
  case MivBitstream::VuhUnitType::V3C_GVD:
    return aau.decGeoFrame;
  case MivBitstream::VuhUnitType::V3C_AVD:
    return aau.decAttrFrame[vuh.vuh_attribute_index()];
  case MivBitstream::VuhUnitType::V3C_PVD:
    return aau.decPckFrame;
  default:
    UNREACHABLE;
  }
}
} // namespace

auto MivDecoder::decodeVideoFrame(MivBitstream::V3cUnitHeader vuh) -> bool {
  auto &decoder = m_videoDecoders[vuh];

  if (!decoder) {
    return pullOutOfBandVideoFrame(vuh);
  }

  fmt::print("Decode video frame: {}, foc={}\n", vuh.summary(), m_au.foc);
  const auto t0 = clockInSeconds();

  const auto atlasIdx = m_au.vps.indexOf(vuh.vuh_atlas_id());
  auto &frame = decFrame(vuh, m_au.atlas[atlasIdx]);
  frame = decoder->getFrame();

  if (frame.empty()) {
    return false;
  }

  m_checker->checkVideoFrame(vuh.vuh_unit_type(), m_au.atlas[atlasIdx].asps, frame);

  m_totalVideoDecodingTime[vuh] += clockInSeconds() - t0;
  return true;
}

auto MivDecoder::pullOutOfBandVideoFrame(MivBitstream::V3cUnitHeader vuh) -> bool {
  PRECONDITION(m_frameServer);

  fmt::print("Pull out-of-band video frame: {}, foc={}\n", vuh.summary(), m_au.foc);

  const auto atlasIdx = m_au.vps.indexOf(vuh.vuh_atlas_id());
  const auto &asps = m_au.atlas[m_au.vps.indexOf(vuh.vuh_atlas_id())].asps;
  auto &frame = decFrame(vuh, m_au.atlas[atlasIdx]);
  frame = m_frameServer(vuh, m_au.foc, m_au.vps, asps);

  if (frame.empty()) {
    return false;
  }

  m_checker->checkVideoFrame(vuh.vuh_unit_type(), m_au.atlas[atlasIdx].asps, frame);
  return true;
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
    vp.viewId = mvpl.mvp_view_id(viewIdx);
    vp.pose = MivBitstream::Pose::decodeFrom(mvpl.camera_extrinsics(viewIdx));
    vp.viewInpaintFlag = mvpl.mvp_inpaint_flag(viewIdx);
    vp.ci = mvpl.camera_intrinsics(viewIdx);
    if (dqParamsPresentFlag) {
      vp.dq = mvpl.depth_quantization(viewIdx);
    }
    if (mvpl.mvp_pruning_graph_params_present_flag()) {
      vp.pp = mvpl.pruning_parent(viewIdx);
    }

    vp.name = fmt::format("pv{:02}", viewIdx);
  }
  m_au.viewParamsList.constructViewIdIndex();
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
    -> Common::Frame<Common::PatchIdx> {
  const auto &asps = m_au.atlas[k].asps;

  const int32_t log2PatchPackingBlockSize = asps.asps_log2_patch_packing_block_size();
  const auto patchPackingBlockSize = 1 << log2PatchPackingBlockSize;
  const auto offset = patchPackingBlockSize - 1;

  const auto atlasBlockToPatchMapWidth = (asps.asps_frame_width() + offset) / patchPackingBlockSize;
  const auto atlasBlockToPatchMapHeight =
      (asps.asps_frame_height() + offset) / patchPackingBlockSize;

  // All elements of TileBlockToPatchMap are first initialized to -1 as follows [9.2.6]
  auto btpm = Common::Frame<Common::PatchIdx>::lumaOnly(
      {atlasBlockToPatchMapWidth, atlasBlockToPatchMapHeight});
  btpm.fillValue(Common::unusedPatchIdx);

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
            btpm.getPlane(0)(yOrg + y, xOrg + x) == Common::unusedPatchIdx) {
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
  const auto atlasId = m_au.vps.vps_atlas_id(k);

  ppl.assign(atdu.atduTotalNumberOfPatches(), {});
  atdu.visit([&](size_t p, MivBitstream::AtduPatchMode /* unused */,
                 const MivBitstream::PatchInformationData &pid) {
    ppl[p] = MivBitstream::PatchParams::decodePdu(pid.patch_data_unit(), m_au.vps, atlasId, asps,
                                                  afps, ath);
  });

  return ppl;
}
} // namespace TMIV::Decoder
