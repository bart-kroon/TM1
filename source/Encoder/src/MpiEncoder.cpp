/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include <TMIV/Encoder/MpiEncoder.h>

#include <TMIV/Common/Factory.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Common/Thread.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/Formatters.h>

namespace TMIV::Encoder {
namespace {
auto createBlockToPatchMap(size_t k, EncoderParams &params) -> Common::Frame<Common::PatchIdx> {
  const auto &asps = params.atlas[k].asps;
  const auto &ppl = params.patchParamsList;

  const auto atlasId = params.vps.vps_atlas_id(k);
  const auto atlasBlockToPatchMapWidth = asps.asps_frame_width();
  const auto atlasBlockToPatchMapHeight = asps.asps_frame_height();

  auto btpm = Common::Frame<Common::PatchIdx>::lumaOnly(
      {atlasBlockToPatchMapWidth, atlasBlockToPatchMapHeight});

  std::fill(btpm.getPlane(0).begin(), btpm.getPlane(0).end(), Common::unusedPatchIdx);

  for (size_t p = 0; p < ppl.size(); ++p) {
    const auto &pp = ppl[p];

    if (pp.atlasId() == atlasId) {
      const auto xOrg = static_cast<size_t>(pp.atlasPatch2dPosX());
      const auto yOrg = static_cast<size_t>(pp.atlasPatch2dPosY());
      const auto atlasPatchWidthBlk = static_cast<size_t>(pp.atlasPatch2dSizeX());
      const auto atlasPatchHeightBlk = static_cast<size_t>(pp.atlasPatch2dSizeY());

      for (size_t y = 0; y < atlasPatchHeightBlk; ++y) {
        for (size_t x = 0; x < atlasPatchWidthBlk; ++x) {
          if (!asps.asps_patch_precedence_order_flag() ||
              btpm.getPlane(0)(yOrg + y, xOrg + x) == Common::unusedPatchIdx) {
            btpm.getPlane(0)(yOrg + y, xOrg + x) = static_cast<uint16_t>(p);
          }
        }
      }
    }
  }

  return btpm;
}

auto dilateTextureAtlas(Common::Frame<> &textureAtlas,
                        const Common::Frame<uint8_t> &transparencyAtlas, uint32_t textureDilation)
    -> Common::Frame<> {
  const auto w = textureAtlas.getWidth();
  const auto h = textureAtlas.getHeight();

  auto texturePrev = expandTexture(textureAtlas);
  auto textureNext = expandTexture(textureAtlas);

  auto transparencyPrev = transparencyAtlas.getPlane(0);
  auto transparencyNext = transparencyPrev;

  static const std::array<Common::Vec2i, 8> offsetList = {
      Common::Vec2i({-1, -1}), Common::Vec2i({0, -1}), Common::Vec2i({1, -1}),
      Common::Vec2i({-1, 0}),  Common::Vec2i({1, 0}),  Common::Vec2i({-1, 1}),
      Common::Vec2i({0, 1}),   Common::Vec2i({1, 1})};

  for (auto iter = 0U; iter < textureDilation; iter++) {
    std::swap(transparencyPrev, transparencyNext);
    std::swap(texturePrev, textureNext);

    Common::parallelFor(w, h, [&](size_t row, size_t col) {
      int32_t cnt = 0;
      Common::Vec3f yuv{};
      if (transparencyPrev(row, col) == 0) {
        for (auto neighbour : offsetList) {
          auto x = static_cast<int32_t>(col) + neighbour.x();
          auto y = static_cast<int32_t>(row) + neighbour.y();
          if ((0 <= x) && (x < w) && (0 <= y) && (y < h)) {
            if (0 < transparencyPrev(y, x)) {
              cnt++;
              yuv += texturePrev(y, x);
            }
          }
        }
      }
      if (0 < cnt) {
        textureNext(row, col) = yuv / static_cast<float>(cnt);
        transparencyNext(row, col) = 255;
      } else {
        textureNext(row, col) = texturePrev(row, col);
        transparencyNext(row, col) = transparencyPrev(row, col);
      }
    });
  }

  return yuv420(quantizeTexture(textureNext, textureAtlas.getBitDepth()));
}

auto reshapeTransparencyAtlas(Common::Frame<uint8_t> &transparencyAtlas,
                              uint32_t transparencyDynamic) -> Common::Frame<> {
  const auto maxInputValue = static_cast<float>(Common::maxLevel(transparencyAtlas.getBitDepth()));
  const auto maxOutputValue = static_cast<float>((uint64_t{1} << transparencyDynamic) - 1);
  const auto maxStorageValue = 1023.F;

  auto transparencyAtlasReshaped = Common::Frame<>::lumaOnly(transparencyAtlas.getSize(), 10);

  std::transform(transparencyAtlas.getPlane(0).begin(), transparencyAtlas.getPlane(0).end(),
                 transparencyAtlasReshaped.getPlane(0).begin(), [&](auto v) {
                   const auto val_in = static_cast<float>(v);
                   const float val_out =
                       maxStorageValue *
                       std::floor(val_in / maxInputValue * maxOutputValue + 0.5F) / maxOutputValue;

                   return static_cast<uint16_t>(val_out);
                 });

  return transparencyAtlasReshaped;
}
} // namespace

MpiEncoder::MpiEncoder(const Common::Json &rootNode, const Common::Json &componentNode)
    : m_rootNode{rootNode}
    , m_intraPeriod{rootNode.require("intraPeriod").as<int32_t>()}
    , m_blockSizeDepthQualityDependent{rootNode.require("blockSizeDepthQualityDependent")
                                           .asVec<int32_t, 2>()}
    , m_textureDilation{componentNode.require("textureDilation").as<uint32_t>()}
    , m_transparencyDynamic{componentNode.require("transparencyDynamic").as<uint32_t>()}
    , m_packer{Common::create<Packer::IPacker>("Packer", rootNode, componentNode)} {
  VERIFY(m_intraPeriod <= maxIntraPeriod);

  // Enforce user-specified atlas size
  auto node = rootNode.require("overrideAtlasFrameSizes");
  for (const auto &subnode : node.as<Common::Json::Array>()) {
    m_overrideAtlasFrameSizes.push_back(subnode.asVec<int32_t, 2>());
  }
}

void MpiEncoder::prepareSequence(const MivBitstream::SequenceConfig &sequenceConfig) {
  m_params = {};
  auto &vps = m_params.vps;

  vps.profile_tier_level()
      .ptl_level_idc(MivBitstream::PtlLevelIdc::Level_3_5)
      .ptl_profile_codec_group_idc(MivBitstream::PtlProfileCodecGroupIdc::HEVC_Main10)
      .ptl_profile_reconstruction_idc(MivBitstream::PtlProfileReconstructionIdc::Rec_Unconstrained)
      .ptl_profile_toolset_idc(MivBitstream::PtlProfileToolsetIdc::MIV_Extended)
      .ptl_profile_toolset_constraints_information([]() {
        auto ptci = MivBitstream::ProfileToolsetConstraintsInformation{};
        ptci.ptc_restricted_geometry_flag(true);
        return ptci;
      }());

  VERIFY_MIVBITSTREAM(!m_overrideAtlasFrameSizes.empty());
  vps.vps_atlas_count_minus1(static_cast<uint8_t>(m_overrideAtlasFrameSizes.size() - 1));

  for (size_t k = 0; k < m_overrideAtlasFrameSizes.size(); ++k) {
    const auto j = MivBitstream::AtlasId{static_cast<uint8_t>(k)};
    vps.vps_atlas_id(k, j)
        .vps_frame_width(j, m_overrideAtlasFrameSizes[k].x())
        .vps_frame_height(j, m_overrideAtlasFrameSizes[k].y())
        .vps_attribute_video_present_flag(j, true);

    vps.attribute_information(j).ai_attribute_count(2);

    static constexpr auto textureBitDepth = 10;
    static constexpr auto transparencyBitDepth = 10;

    vps.attribute_information(j)
        .ai_attribute_type_id(0, MivBitstream::AiAttributeTypeId::ATTR_TEXTURE)
        .ai_attribute_dimension_minus1(0, 2)
        .ai_attribute_2d_bit_depth_minus1(0, textureBitDepth - 1);

    vps.attribute_information(j)
        .ai_attribute_type_id(1, MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY)
        .ai_attribute_dimension_minus1(1, 0)
        .ai_attribute_2d_bit_depth_minus1(1, transparencyBitDepth - 1);
  }

  m_params.viewParamsList = sequenceConfig.sourceViewParams();
  m_frameRate = sequenceConfig.frameRate;

  auto depthLowQualityFlag = false;

  if (const auto &node = m_rootNode.optional("depthLowQualityFlag")) {
    depthLowQualityFlag = node.as<bool>();
    m_params.casps.casps_miv_extension().casme_depth_low_quality_flag(depthLowQualityFlag);
  }

  if (const auto &subnode = m_rootNode.optional("ViewingSpace")) {
    m_params.viewingSpace = MivBitstream::ViewingSpace::loadFromJson(subnode, m_rootNode);
  }
  if (m_rootNode.require("viewportCameraParametersSei").as<bool>()) {
    m_params.viewportCameraParameters = MivBitstream::ViewportCameraParameters::fromViewParams(
        sequenceConfig.cameraByName("viewport").viewParams);
  }
  if (m_rootNode.require("viewportPositionSei").as<bool>()) {
    m_params.viewportPosition = MivBitstream::ViewportPosition::fromViewParams(
        sequenceConfig.cameraByName("viewport").viewParams);
  }

  m_blockSize = m_blockSizeDepthQualityDependent[depthLowQualityFlag ? 1U : 0U];
  VERIFY(2 <= m_blockSize);
  VERIFY((m_blockSize & (m_blockSize - 1)) == 0);

  // Group atlases together to restrict atlas-level sub-bitstream access
  auto &gm = vps.vps_miv_extension().group_mapping();
  gm.gm_group_count(1);
  for (size_t i = 0; i < m_overrideAtlasFrameSizes.size(); ++i) {
    gm.gm_group_id(i, 0);
  }

  m_params.casps.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4())
      .casps_miv_extension()
      .casme_depth_quantization_params_present_flag(true)
      .vui_parameters(vuiParameters());

  // NOTE(FT, m55089): need to have only non basic views to allow for splitting the patches
  for (auto &v : m_params.viewParamsList) {
    v.isBasicView = false;
  }

  setGiGeometry3dCoordinatesBitdepthMinus1();

  // Set-up ASPS and AFPS
  prepareIvau();
}

namespace {
void setTile(EncoderParams &params) {
  params.tileParamsLists.clear();
  params.tileParamsLists.resize(params.atlas.size());
  // const bool singleTileInAtlasFrameFlag = true;
  for (size_t atlasIdx = 0; atlasIdx < params.atlas.size(); ++atlasIdx) {
    MivBitstream::TilePartition t;
    t.partitionHeight(params.atlas[atlasIdx].asps.asps_frame_height());
    t.partitionWidth(params.atlas[atlasIdx].asps.asps_frame_width());
    t.partitionPosX(0);
    t.partitionPosY(0);
    params.tileParamsLists[atlasIdx].emplace_back(t);

    auto afti = TMIV::MivBitstream::AtlasFrameTileInformation{};
    afti.afti_single_tile_in_atlas_frame_flag(true).afti_num_tiles_in_atlas_frame_minus1(0);
    params.atlas[atlasIdx].afps.atlas_frame_tile_information(afti);
  }
  // set ATH
  for (size_t atlasIdx = 0; atlasIdx < params.tileParamsLists.size(); ++atlasIdx) {
    for (uint8_t tileIdx = 0;
         tileIdx < Common::downCast<uint8_t>(params.tileParamsLists[atlasIdx].size()); ++tileIdx) {
      params.atlas[atlasIdx].athList.clear();
      auto &atlas = params.atlas[atlasIdx];
      auto ath = MivBitstream::AtlasTileHeader{};
      ath.ath_type(MivBitstream::AthType::I_TILE)
          .ath_ref_atlas_frame_list_asps_flag(true)
          .ath_pos_min_d_quantizer(uint8_t{});
      ath.ath_id(tileIdx);
      atlas.athList.push_back(ath);
    }
  }
}

void assignPatchesToTiles(EncoderParams &params) {
  size_t patchNum = params.patchParamsList.size();
  size_t tilePatchNum = 0;
  for (const auto &patch : params.patchParamsList) {
    size_t atlasID = (patch.atlasId() == TMIV::MivBitstream::AtlasId(0)) ? 0 : 1;
    auto patchPosX = patch.atlasPatch2dPosX();
    auto patchPosY = patch.atlasPatch2dPosY();
    auto patchSizeX = patch.atlasPatch2dSizeX();
    auto patchSizeY = patch.atlasPatch2dSizeY();
    for (auto &tile : params.tileParamsLists[atlasID]) {
      auto tilePosX = tile.partitionPosX();
      auto tilePosY = tile.partitionPosY();
      auto tileSizeX = tile.partitionWidth();
      auto tileSizeY = tile.partitionHeight();

      if (patchPosX >= tilePosX && patchPosY >= tilePosY &&
          (patchPosX + patchSizeX) <= (tilePosX + tileSizeX) &&
          (patchPosY + patchSizeY) <= (tilePosY + tileSizeY)) {
        tile.addPatchToTile(patch);
        ++tilePatchNum;
        break;
      }
    }
  }

  POSTCONDITION(tilePatchNum == patchNum);
}
} // namespace

auto MpiEncoder::processAccessUnit(int32_t firstFrameId, int32_t lastFrameId)
    -> const EncoderParams & {
  LIMITATION(m_params.viewParamsList.size() == 1);
  const auto &mpiViewParams = m_params.viewParamsList.front();
  Common::Vec2i mpiSize{
      static_cast<int32_t>(mpiViewParams.ci.ci_projection_plane_width_minus1()) + 1,
      static_cast<int32_t>(mpiViewParams.ci.ci_projection_plane_height_minus1()) + 1};

  m_mpiFrameBuffer.clear();

  for (int32_t frameIdx = firstFrameId; frameIdx < lastFrameId; frameIdx++) {
    m_mpiFrameBuffer.emplace_back(readFrame(frameIdx));
  }

  m_params.patchParamsList.clear();

  auto aggregatedMask = Common::Frame<uint8_t>::lumaOnly(mpiSize);

  Common::FrameList<> pixelLayerIndicesPerFrame(lastFrameId - firstFrameId,
                                                Common::Frame<>::lumaOnly(mpiSize));
  size_t nbActivePixels{};

  setTile(m_params);
  m_packer->initialize(m_params.tileParamsLists);
  m_overrideTileFrameSizes = std::vector<Common::SizeVector>(m_params.atlas.size());
  for (size_t atlasIdx = 0; atlasIdx < m_params.atlas.size(); ++atlasIdx) {
    auto &tileSizes = m_overrideTileFrameSizes[atlasIdx];
    tileSizes = Common::SizeVector(m_params.tileParamsLists[atlasIdx].size());
    for (size_t tileIdx = 0; tileIdx < m_params.tileParamsLists[atlasIdx].size(); ++tileIdx) {
      tileSizes[tileIdx] =
          Common::Vec2i{m_params.tileParamsLists[atlasIdx][tileIdx].partitionWidth(),
                        m_params.tileParamsLists[atlasIdx][tileIdx].partitionHeight()};
    }
  }
  m_packer->initialize(m_overrideTileFrameSizes, m_blockSize);

  for (auto layerId = 0; layerId < mpiViewParams.nbMpiLayers; ++layerId) {
    aggregatedMask.fillZero();

    for (size_t frameBufferIdx = 0; frameBufferIdx < pixelLayerIndicesPerFrame.size();
         ++frameBufferIdx) {
      const auto &frame = m_mpiFrameBuffer[frameBufferIdx];
      auto &pixelLayerIndices = pixelLayerIndicesPerFrame[frameBufferIdx];

      Common::parallelFor(frame.getPixelList().size(), [&](size_t pixelId) {
        const auto &pixel = frame.getPixelList()[pixelId];
        auto &pixelLayerIdx = pixelLayerIndices.getPlane(0)[pixelId];

        if (pixelLayerIdx < pixel.size()) {
          const auto &attribute = pixel[pixelLayerIdx];

          if (attribute.geometry == layerId) {
            aggregatedMask.getPlane(0)[pixelId] = 255;
            pixelLayerIdx++;
          }
        }
      });
    }

    nbActivePixels +=
        std::count_if(aggregatedMask.getPlane(0).begin(), aggregatedMask.getPlane(0).end(),
                      [](auto x) { return (x > 0); });

    auto patchParamsListLayer = m_packer->pack(m_overrideTileFrameSizes, {aggregatedMask},
                                               m_params.viewParamsList, m_blockSize, {});

    for (auto &patchParams : patchParamsListLayer) {
      patchParams.atlasPatch3dOffsetD(layerId);

      // NOTE(BK): I do not understand why asps_2d_geometry_bit_depth_minus1 != 0, but in that case
      // we need to calculate atlasPatch3drangeD.
      const auto atlasIdx = m_params.vps.indexOf(patchParams.atlasId());
      const auto bitDepth = m_params.atlas[atlasIdx].asps.asps_geometry_2d_bit_depth_minus1() + 1U;
      patchParams.atlasPatch3dRangeD(Common::maxLevel(bitDepth));
    }

    std::move(patchParamsListLayer.begin(), patchParamsListLayer.end(),
              back_inserter(m_params.patchParamsList));
  }

  Common::logInfo("Aggregated luma samples per frame is {}M",
                  1e-6 * 2 * static_cast<double>(nbActivePixels));
  m_maxLumaSamplesPerFrame = std::max(m_maxLumaSamplesPerFrame, nbActivePixels);

  Common::logInfo("Packing done with nb of patches = {}", m_params.patchParamsList.size());
  assignPatchesToTiles(m_params);
  m_blockToPatchMapPerAtlas.clear();

  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    m_blockToPatchMapPerAtlas.emplace_back(createBlockToPatchMap(k, m_params));
  }

  Common::logInfo("Block to patch map created");

  return m_params;
}

auto MpiEncoder::popAtlas() -> Common::V3cFrameList {
  const auto &ppl = m_params.patchParamsList;
  const auto &mpiFrame = m_mpiFrameBuffer.front();
  Common::V3cFrameList atlasList;

  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    const auto atlasId = m_params.vps.vps_atlas_id(k);
    const auto frameWidth = m_params.vps.vps_frame_width(atlasId);
    const auto frameHeight = m_params.vps.vps_frame_height(atlasId);

    auto textureFrame = Common::Frame<>::yuv444({frameWidth, frameHeight}, 10);
    auto transparencyFrame = Common::Frame<uint8_t>::lumaOnly({frameWidth, frameHeight});

    textureFrame.fillNeutral();
    transparencyFrame.fillZero();

    const auto &blockToPatchMap = m_blockToPatchMapPerAtlas[k];

    Common::parallelFor(frameWidth, frameHeight, [&](size_t i, size_t j) {
      if (auto patchIdx = blockToPatchMap.getPlane(0)(i, j); patchIdx != Common::unusedPatchIdx) {
        const auto &patch = ppl[patchIdx];
        auto posInView = patch.atlasToView({static_cast<int32_t>(j), static_cast<int32_t>(i)});

        const auto &pixel = mpiFrame(posInView.y(), posInView.x());
        auto layerId = static_cast<uint16_t>(patch.atlasPatch3dOffsetD());

        auto *const iter =
            std::lower_bound(pixel.begin(), pixel.end(), layerId,
                             [](auto pixel_, auto layerId_) { return pixel_.geometry < layerId_; });

        if (iter != pixel.end() && iter->geometry == layerId) {
          textureFrame.getPlane(0)(i, j) = iter->texture[0];
          textureFrame.getPlane(1)(i, j) = iter->texture[1];
          textureFrame.getPlane(2)(i, j) = iter->texture[2];
          transparencyFrame.getPlane(0)(i, j) = iter->transparency;
        }
      }
    });

    auto textureAtlas = dilateTextureAtlas(textureFrame, transparencyFrame, m_textureDilation);
    auto transparencyAtlas = reshapeTransparencyAtlas(transparencyFrame, m_transparencyDynamic);

    atlasList.emplace_back().texture = std::move(textureAtlas);
    atlasList.back().transparency = std::move(transparencyAtlas);
  }

  m_mpiFrameBuffer.pop_front();

  return atlasList;
}

auto MpiEncoder::vuiParameters() const -> MivBitstream::VuiParameters {
  auto numUnitsInTick = 1;
  auto timeScale = static_cast<int32_t>(numUnitsInTick * m_frameRate);
  LIMITATION(timeScale == numUnitsInTick * m_frameRate);

  auto vui = MivBitstream::VuiParameters{};
  vui.vui_num_units_in_tick(numUnitsInTick)
      .vui_time_scale(timeScale)
      .vui_poc_proportional_to_timing_flag(false)
      .vui_hrd_parameters_present_flag(false);
  vui.vui_unit_in_metres_flag(true);
  vui.coordinate_system_parameters() = {};
  return vui;
}

void MpiEncoder::setGiGeometry3dCoordinatesBitdepthMinus1() {
  uint8_t numBitsMinus1 = 9; // Main 10
  for (auto &vp : m_params.viewParamsList) {
    const auto size = std::max(vp.ci.ci_projection_plane_width_minus1() + 1,
                               vp.ci.ci_projection_plane_height_minus1() + 1);
    numBitsMinus1 = std::max(numBitsMinus1, static_cast<uint8_t>(Common::ceilLog2(size) - 1));
  }
  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_params.vps.vps_atlas_id(k);
    m_params.vps.geometry_information(j).gi_geometry_3d_coordinates_bit_depth_minus1(numBitsMinus1);
  }
}

void MpiEncoder::prepareIvau() {
  m_params.atlas.resize(m_params.vps.vps_atlas_count_minus1() + size_t{1});

  auto numBitsMinus1 =
      static_cast<uint8_t>(Common::ceilLog2(m_params.viewParamsList.front().nbMpiLayers) - 1);

  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    auto &atlas = m_params.atlas[k];
    const auto j = m_params.vps.vps_atlas_id(k);
    const auto &gi = m_params.vps.geometry_information(j);

    // Set ASPS parameters
    atlas.asps.asps_frame_width(m_params.vps.vps_frame_width(j))
        .asps_frame_height(m_params.vps.vps_frame_height(j))
        .asps_geometry_3d_bit_depth_minus1(gi.gi_geometry_3d_coordinates_bit_depth_minus1())
        .asps_geometry_2d_bit_depth_minus1(numBitsMinus1)
        .asps_log2_max_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4())
        .asps_use_eight_orientations_flag(true)
        .asps_extended_projection_enabled_flag(true)
        .asps_normal_axis_limits_quantization_enabled_flag(true)
        .asps_max_number_projections_minus1(
            static_cast<uint16_t>(m_params.viewParamsList.size() - 1))
        .asps_log2_patch_packing_block_size(Common::ceilLog2(m_blockSize))
        .asps_num_ref_atlas_frame_lists_in_asps(1)
        .asps_miv_extension()
        .asme_patch_constant_depth_flag(true);
  }
}

auto MpiEncoder::log2FocLsbMinus4() const -> uint8_t {
  // Avoid confusion but test MSB/LSB logic in decoder
  return Common::downCast<uint8_t>(std::max(4U, Common::ceilLog2(m_intraPeriod) + 1U) - 4U);
}
} // namespace TMIV::Encoder
