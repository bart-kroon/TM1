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

#include "EncoderImpl.h"
#include "GeometryQuantizer.h"

#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/SequenceConfig.h>

#include <numeric>

namespace TMIV::Encoder {
namespace {
[[nodiscard]] auto createProfileTierLevel(const Configuration &config) {
  auto ptl = MivBitstream::ProfileTierLevel{}
                 .ptl_level_idc(config.levelIdc)
                 .ptl_profile_codec_group_idc(config.codecGroupIdc)
                 .ptl_profile_reconstruction_idc(config.reconstructionIdc)
                 .ptl_profile_toolset_idc(config.toolsetIdc);

  if (config.oneV3cFrameOnly) {
    ptl.ptl_profile_toolset_constraints_information(
        MivBitstream::ProfileToolsetConstraintsInformation{}.ptc_one_v3c_frame_only_flag(true));
  }
  return ptl;
}

[[nodiscard]] auto adaptViewParamsList(const Configuration &config,
                                       MivBitstream::ViewParamsList viewParamsList) {
  for (auto &vp : viewParamsList) {
    vp.hasOccupancy |= (!vp.isBasicView && config.patchRedundancyRemoval) || 0 < config.maxEntityId;
  }
  return viewParamsList;
}

[[nodiscard]] auto calculateViewGridSize(const Configuration &config,
                                         const MivBitstream::ViewParamsList &viewParamsList,
                                         bool depthLowQualityFlag) {
  int32_t x{};
  int32_t y{};

  const auto blockSize = config.blockSize(depthLowQualityFlag);

  for (const auto &viewParams : viewParamsList) {
    x = std::max(x, (viewParams.ci.ci_projection_plane_width_minus1() + blockSize) / blockSize);
    y = std::max(y, (viewParams.ci.ci_projection_plane_height_minus1() + blockSize) / blockSize);
  }
  return Common::Vec2i{x, y};
}

// Calculate atlas frame sizes [MPEG/M52994 v2]
[[nodiscard]] auto
calculateNominalAtlasFrameSizes(const Configuration &config,
                                const MivBitstream::ViewParamsList &viewParamsList,
                                double frameRate, bool depthLowQualityFlag) {
  if (config.oneViewPerAtlasFlag) {
    // No constraints: one atlas per transport view
    auto result = Common::SizeVector(viewParamsList.size());
    std::transform(std::cbegin(viewParamsList), std::cend(viewParamsList), std::begin(result),
                   [](const MivBitstream::ViewParams &x) { return x.ci.projectionPlaneSize(); });
    return result;
  }

  if (!config.overrideAtlasFrameSizes.empty()) {
    Common::logWarning("When overriding nominal atlas frame sizes, constraints are not checked.");
    return config.overrideAtlasFrameSizes;
  }

  const auto lumaSamplesPerAtlasSample =
      (config.haveTexture ? 1. : 0.) +
      (config.haveGeometry ? (config.geometryScaleEnabledFlag ? 0.25 : 1.) : 0.);
  const auto numGroups = std::max(1.F, static_cast<float>(config.numGroups));

  const auto blockSize = config.blockSize(depthLowQualityFlag);
  const auto maxBlockRate =
      config.maxLumaSampleRate / (numGroups * lumaSamplesPerAtlasSample * Common::sqr(blockSize));

  // Translate block rate into a maximum number of blocks
  const auto maxBlocks = static_cast<int32_t>(maxBlockRate / frameRate);

  // Calculate the number of atlases
  const auto maxBlocksPerAtlas1 = config.maxLumaPictureSize / Common::sqr(blockSize);
  auto numAtlases = (maxBlocks + maxBlocksPerAtlas1 - 1) / maxBlocksPerAtlas1;

  if (numAtlases > config.maxAtlases) {
    Common::logInfo("The maxAtlases constraint is a limiting factor.");
    numAtlases = config.maxAtlases;
  }

  // Calculate the number of blocks per atlas
  auto maxBlocksPerAtlas2 = maxBlocks / numAtlases;

  if (maxBlocksPerAtlas2 > maxBlocksPerAtlas1) {
    Common::logInfo("The maxLumaPictureSize constraint is a limiting factor.");
    maxBlocksPerAtlas2 = maxBlocksPerAtlas1;
  }

  // Take the smallest reasonable width
  const auto viewGridSize = calculateViewGridSize(config, viewParamsList, depthLowQualityFlag);
  const auto atlasGridWidth = viewGridSize.x();
  const auto atlasGridHeight = maxBlocksPerAtlas2 / atlasGridWidth;

  // Warn if the aspect ratio is outside of HEVC limits (unlikely)
  if (atlasGridWidth * 8 < atlasGridHeight || atlasGridHeight * 8 < atlasGridWidth) {
    Common::logWarning("Atlas aspect ratio is outside of HEVC general tier and level limits");
  }

  return Common::SizeVector(numAtlases, {atlasGridWidth * blockSize, atlasGridHeight * blockSize});
}

[[nodiscard]] auto createGeometryInformation(uint32_t bitDepth,
                                             const MivBitstream::ViewParamsList &viewParamsList) {
  const auto bitDepthMinus1 = Common::downCast<uint8_t>(bitDepth - 1);

  auto gi = MivBitstream::GeometryInformation{};
  gi.gi_geometry_2d_bit_depth_minus1(bitDepthMinus1)
      .gi_geometry_3d_coordinates_bit_depth_minus1(std::accumulate(
          viewParamsList.cbegin(), viewParamsList.cend(), bitDepthMinus1,
          [](uint8_t init, const MivBitstream::ViewParams &vp) {
            const auto viewWidth = vp.ci.ci_projection_plane_width_minus1() + 1;
            const auto viewHeight = vp.ci.ci_projection_plane_height_minus1() + 1;
            const auto requiredBitDepth = Common::ceilLog2(std::max(viewWidth, viewHeight));
            return std::max(init, Common::downCast<uint8_t>(requiredBitDepth - 1));
          }));

  return gi;
}

[[nodiscard]] auto createOccupancyInformation(uint32_t bitDepth) {
  auto oi = MivBitstream::OccupancyInformation{};
  oi.oi_occupancy_codec_id(0)
      .oi_lossy_occupancy_compression_threshold(128)
      .oi_occupancy_2d_bit_depth_minus1(Common::downCast<uint8_t>(bitDepth - 1))
      .oi_occupancy_MSB_align_flag(false);
  return oi;
}

[[nodiscard]] auto createAttributeInformation(uint32_t bitDepth) {
  auto ai = MivBitstream::AttributeInformation{};
  ai.ai_attribute_count(1)
      .ai_attribute_type_id(0, MivBitstream::AiAttributeTypeId::ATTR_TEXTURE)
      .ai_attribute_dimension_minus1(0, 2)
      .ai_attribute_2d_bit_depth_minus1(0, Common::downCast<uint8_t>(bitDepth - 1));
  return ai;
}

[[nodiscard]] auto createGroupMapping(uint8_t numGroups,
                                      const Common::SizeVector &atlasFrameSizes) {
  auto gm = MivBitstream::GroupMapping{};
  gm.gm_group_count(numGroups);

  if (0 < numGroups) {
    for (size_t i = 0; i < atlasFrameSizes.size(); ++i) {
      gm.gm_group_id(i, 0);
    }
  }
  return gm;
}

[[nodiscard]] auto createVpsMiv2Extension(const Configuration &config,
                                          const Common::SizeVector &atlasFrameSizes) {
  auto vme2 = MivBitstream::VpsMiv2Extension{};
  auto &vme = vme2.vps_miv_extension();
  vme.vme_embedded_occupancy_enabled_flag(config.embeddedOccupancy).group_mapping() =
      createGroupMapping(config.numGroups, atlasFrameSizes);

  if (!config.embeddedOccupancy) {
    vme.vme_occupancy_scale_enabled_flag(config.haveOccupancy);
  }
  vme2.vme_patch_margin_enabled_flag(true);
  return vme2;
}

[[nodiscard]] auto createV3cParameterSet(const Configuration &config,
                                         const MivBitstream::ViewParamsList &viewParamsList,
                                         double frameRate, bool depthLowQualityFlag) {
  const auto atlasFrameSizes =
      calculateNominalAtlasFrameSizes(config, viewParamsList, frameRate, depthLowQualityFlag);

  auto vps = MivBitstream::V3cParameterSet{};
  vps.profile_tier_level(createProfileTierLevel(config))
      .vps_atlas_count_minus1(static_cast<uint8_t>(atlasFrameSizes.size() - 1));

  for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
    const auto j = MivBitstream::AtlasId{k};
    vps.vps_atlas_id(k, j)
        .vps_frame_width(j, atlasFrameSizes[k].x())
        .vps_frame_height(j, atlasFrameSizes[k].y())
        .vps_geometry_video_present_flag(j, config.haveGeometry)
        .vps_occupancy_video_present_flag(j, config.haveOccupancy)
        .vps_attribute_video_present_flag(j, config.haveTexture)
        .vps_miv_2_extension(createVpsMiv2Extension(config, atlasFrameSizes));

    if (config.haveOccupancy) {
      vps.occupancy_information(j, createOccupancyInformation(config.occBitDepth));
    }
    if (config.haveGeometry) {
      vps.geometry_information(j, createGeometryInformation(config.geoBitDepth, viewParamsList));
    }
    if (config.haveTexture) {
      vps.attribute_information(j, createAttributeInformation(config.texBitDepth));
    }
  }
  return vps;
}

[[nodiscard]] auto log2FocLsbMinus4(int32_t intraPeriod) {
  // Avoid confusion but test MSB/LSB logic in decoder
  return Common::downCast<uint8_t>(std::max(4U, Common::ceilLog2(intraPeriod) + 1U) - 4U);
}

[[nodiscard]] auto vuiParameters(const MivBitstream::SequenceConfig &sequenceConfig) {
  auto numUnitsInTick = 1;
  auto timeScale = static_cast<int32_t>(numUnitsInTick * sequenceConfig.frameRate);
  LIMITATION(timeScale == numUnitsInTick * sequenceConfig.frameRate);

  auto vui = MivBitstream::VuiParameters{};
  vui.vui_num_units_in_tick(numUnitsInTick)
      .vui_time_scale(timeScale)
      .vui_poc_proportional_to_timing_flag(false)
      .vui_hrd_parameters_present_flag(false);
  vui.vui_unit_in_metres_flag(sequenceConfig.lengthsInMeters);
  vui.coordinate_system_parameters() = {};
  return vui;
}

[[nodiscard]] auto
createCommonAtlasSequenceParameterSet(const Configuration &config,
                                      const MivBitstream::SequenceConfig &sequenceConfig,
                                      bool depthLowQualityFlag) {
  auto casps = MivBitstream::CommonAtlasSequenceParameterSetRBSP{};
  casps.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4(config.intraPeriod))
      .casps_miv_extension()
      .casme_depth_low_quality_flag(depthLowQualityFlag)
      .casme_depth_quantization_params_present_flag(config.dqParamsPresentFlag)
      .vui_parameters(vuiParameters(sequenceConfig));
  return casps;
}

[[nodiscard]] auto patchSizeQuantizers(const Configuration &config,
                                       const MivBitstream::ViewParamsList &viewParamsList,
                                       bool depthLowQualityFlag) {
  auto quantizer = config.blockSize(depthLowQualityFlag);

  for (const auto &vp : viewParamsList) {
    quantizer = std::gcd(quantizer, vp.ci.ci_projection_plane_width_minus1() + 1);
    quantizer = std::gcd(quantizer, vp.ci.ci_projection_plane_height_minus1() + 1);
  }

  // NOTE(BK): There may be rotated patches of full width or height: same quantizer for x and y
  return Common::Vec2i{quantizer, quantizer};
}

[[nodiscard]] auto createAspsMivExtension(const Configuration &config,
                                          const MivBitstream::V3cParameterSet &vps,
                                          int32_t blockSize, MivBitstream::AtlasId j) {
  auto asme = MivBitstream::AspsMivExtension{};
  asme.asme_embedded_occupancy_enabled_flag(config.embeddedOccupancy)
      .asme_patch_texture_offset_enabled_flag(config.textureOffsetFlag)
      .asme_max_entity_id(config.maxEntityId);

  if (config.haveOccupancy) {
    const auto occFrameSizeX = std::lcm(2, vps.vps_frame_width(j) / blockSize);
    const auto occFrameSizeY = std::lcm(2, vps.vps_frame_height(j) / blockSize);
    const auto scaleFactorX = vps.vps_frame_width(j) / occFrameSizeX;
    const auto scaleFactorY = vps.vps_frame_height(j) / occFrameSizeY;
    asme.asme_occupancy_scale_factor_x_minus1(Common::downCast<uint16_t>(scaleFactorX - 1))
        .asme_occupancy_scale_factor_y_minus1(Common::downCast<uint16_t>(scaleFactorY - 1));
  }
  if (config.textureOffsetFlag) {
    asme.asme_patch_texture_offset_bit_depth_minus1(
        Common::downCast<uint16_t>(config.textureOffsetBitCount - 1));
  }
  return asme;
}

[[nodiscard]] auto createAspsMiv2Extension(const Configuration &config) {
  auto asme2 = MivBitstream::AspsMiv2Extension{};
  asme2.asme_patch_margin_enabled_flag(config.patchMarginFlag);

  return asme2;
}

[[nodiscard]] auto
createAtlasSequenceParameterSet(const Configuration &config,
                                const MivBitstream::V3cParameterSet &vps,
                                const MivBitstream::ViewParamsList &viewParamsList,
                                bool depthLowQualityFlag, MivBitstream::AtlasId j) {
  const auto blockSize = config.blockSize(depthLowQualityFlag);
  const auto psq = patchSizeQuantizers(config, viewParamsList, depthLowQualityFlag);

  auto asps = MivBitstream::AtlasSequenceParameterSetRBSP{};
  asps.asps_frame_width(vps.vps_frame_width(j))
      .asps_frame_height(vps.vps_frame_height(j))
      .asps_log2_max_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4(config.intraPeriod))
      .asps_use_eight_orientations_flag(true)
      .asps_extended_projection_enabled_flag(true)
      .asps_normal_axis_limits_quantization_enabled_flag(true)
      .asps_max_number_projections_minus1(viewParamsList.maxViewIdValue())
      .asps_log2_patch_packing_block_size(Common::ceilLog2(blockSize))
      .asps_num_ref_atlas_frame_lists_in_asps(1)
      .asps_patch_size_quantizer_present_flag(psq.x() != blockSize || psq.y() != blockSize)
      .asps_miv_extension() = createAspsMivExtension(config, vps, blockSize, j);
  asps.asps_miv_2_extension() = createAspsMiv2Extension(config);

  if (vps.vps_geometry_video_present_flag(j)) {
    const auto &gi = vps.geometry_information(j);
    asps.asps_geometry_3d_bit_depth_minus1(gi.gi_geometry_3d_coordinates_bit_depth_minus1())
        .asps_geometry_2d_bit_depth_minus1(gi.gi_geometry_2d_bit_depth_minus1());
  }
  return asps;
}

[[nodiscard]] auto createAtlasTileHeader(const Configuration &config,
                                         const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                                         const MivBitstream::ViewParamsList &viewParamsList,
                                         bool depthLowQualityFlag) {
  auto ath = MivBitstream::AtlasTileHeader{};
  ath.ath_type(MivBitstream::AthType::I_TILE)
      .ath_ref_atlas_frame_list_asps_flag(true)
      .ath_pos_min_d_quantizer(asps.asps_geometry_3d_bit_depth_minus1() + 1);

  if (asps.asps_patch_size_quantizer_present_flag()) {
    const auto psq = patchSizeQuantizers(config, viewParamsList, depthLowQualityFlag);

    ath.ath_patch_size_x_info_quantizer(Common::ceilLog2(psq.x()));
    ath.ath_patch_size_y_info_quantizer(Common::ceilLog2(psq.y()));
  }
  return ath;
}

[[nodiscard]] auto createEncoderAtlasParams(const Configuration &config,
                                            const MivBitstream::V3cParameterSet &vps,
                                            const MivBitstream::ViewParamsList &viewParamsList,
                                            bool depthLowQualityFlag, MivBitstream::AtlasId j) {
  auto atlas = MivBitstream::EncoderAtlasParams{};
  atlas.asps = createAtlasSequenceParameterSet(config, vps, viewParamsList, depthLowQualityFlag, j);
  atlas.athList.push_back(
      createAtlasTileHeader(config, atlas.asps, viewParamsList, depthLowQualityFlag));

  return atlas;
}
} // namespace

[[nodiscard]] auto createEncoderParams(const Configuration &config,
                                       const MivBitstream::SequenceConfig &sequenceConfig,
                                       const MivBitstream::ViewParamsList &viewParamsList,
                                       bool depthLowQualityFlag) -> EncoderParams {
  auto params = EncoderParams{};

  params.viewParamsList = adaptViewParamsList(config, viewParamsList);

  params.vps = createV3cParameterSet(config, params.viewParamsList, sequenceConfig.frameRate,
                                     depthLowQualityFlag);

  params.casps = createCommonAtlasSequenceParameterSet(config, sequenceConfig, depthLowQualityFlag);

  if (config.chromaScaleEnabledFlag) {
    params.casps.casps_miv_2_extension().casme_chroma_scaling_present_flag(
        config.chromaScaleEnabledFlag);
  }

  params.viewingSpace = config.viewingSpace;

  if (config.viewportCameraParametersSei) {
    params.viewportCameraParameters = MivBitstream::ViewportCameraParameters::fromViewParams(
        sequenceConfig.cameraByName("viewport").viewParams);
  }
  if (config.viewportPositionSei) {
    params.viewportPosition = MivBitstream::ViewportPosition::fromViewParams(
        sequenceConfig.cameraByName("viewport").viewParams);
  }

  // Create encoder atlas parameters for all atlases
  params.atlas.resize(params.vps.vps_atlas_count_minus1() + size_t{1});

  for (uint8_t k = 0; k <= params.vps.vps_atlas_count_minus1(); ++k) {
    params.atlas[k] = createEncoderAtlasParams(config, params.vps, params.viewParamsList,
                                               depthLowQualityFlag, params.vps.vps_atlas_id(k));
  }
  return params;
}

namespace {
[[nodiscard]] auto sampleBudget(const MivBitstream::V3cParameterSet &vps) {
  int32_t sampleBudget = 0;
  for (size_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
    const auto j = vps.vps_atlas_id(k);
    sampleBudget += (vps.vps_frame_width(j) * vps.vps_frame_height(j));
  }
  return sampleBudget;
}
} // namespace

void Encoder::Impl::prepareSequence(const SourceUnit &unit) {
  m_blockSize = m_config.blockSize(unit.depthLowQualityFlag);

  m_transportViewParams = unit.viewParamsList;
  m_semiBasicViewCount = unit.semiBasicViewCount;

  m_params = createEncoderParams(m_config, unit.sequenceConfig, m_transportViewParams,
                                 unit.depthLowQualityFlag);

  const auto pruningParents = m_pruner->prepareSequence(
      {params().viewParamsList, unit.depthLowQualityFlag, sampleBudget(params().vps)});

  for (size_t viewIdx = 0; viewIdx < pruningParents.size(); ++viewIdx) {
    m_params.viewParamsList[viewIdx].pp = pruningParents[viewIdx];
  }
}
} // namespace TMIV::Encoder
