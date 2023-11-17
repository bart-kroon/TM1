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

#include <TMIV/Encoder/Encoder.h>

#include <TMIV/Aggregator/IAggregator.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/Formatters.h>
#include <TMIV/MivBitstream/SequenceConfig.h>
#include <TMIV/Packer/IPacker.h>
#include <TMIV/Pruner/IPruner.h>

#include "Configuration.h"
#include "SampleStats.h"

#include <algorithm>
#include <memory>
#include <numeric>

using TMIV::Aggregator::IAggregator;
using TMIV::Packer::IPacker;
using TMIV::Pruner::IPruner;

namespace TMIV::Encoder {
using TextureStats = Common::stack::Vec3<SampleStats>;
using PatchTextureStats = std::vector<TextureStats>;
using MivBitstream::EncoderParams;

class Encoder::Impl {
public:
  explicit Impl(const Common::Json &componentNode)
      : m_pruner{Common::create<Pruner::IPruner>("Pruner", componentNode, componentNode)}
      , m_aggregator{Common::create<IAggregator>("Aggregator", componentNode, componentNode)}
      , m_packer{Common::create<IPacker>("Packer", componentNode, componentNode)}
      , m_config(componentNode) {}

  [[nodiscard]] auto isStart(const SourceUnit & /* unit */) {
    return ++m_lastIdx % m_config.interPeriod == 0;
  }

  void process(std::vector<SourceUnit> buffer, const Common::StageSource<CodableUnit> &source_) {
    if (m_firstIdx == 0) {
      prepareSequence(buffer.front());
    }

    m_params.foc = m_firstIdx % m_config.intraPeriod;

    prepareAccessUnit();

    for (auto &sourceUnit : buffer) {
      pushFrame(std::move(sourceUnit.deepFrameList));
    }

    completeAccessUnit();

    if (m_config.chromaScaleEnabledFlag) {
      scaleChromaDynamicRange();
    }

    constructVideoFrames();

    auto type = m_firstIdx % m_config.intraPeriod == 0 ? MivBitstream::CodableUnitType::IDR
                                                       : MivBitstream::CodableUnitType::TRIAL;

    for (auto &deepFrameList : m_videoFrameBuffer) {
      source_.encode({m_params, std::move(deepFrameList), type});

      type = MivBitstream::CodableUnitType::SKIP;
      // Pattern for intraPeriod=8 and interPeriod=4:
      //    IDR SKIP SKIP SKIP TRIAL SKIP SKIP SKIP IDR ...
      // FOC=0   1    2    3    4     5    6    7    0  ...
    }

    m_videoFrameBuffer.clear();

    m_firstIdx = m_lastIdx;
  }

  [[nodiscard]] auto maxLumaSamplesPerFrame() const { return m_maxLumaSamplesPerFrame; }

private:
  [[nodiscard]] static auto createProfileTierLevel(const Configuration &config) {
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

  [[nodiscard]] static auto adaptViewParamsList(const Configuration &config,
                                                MivBitstream::ViewParamsList viewParamsList) {
    for (auto &vp : viewParamsList) {
      vp.hasOccupancy |=
          (!vp.isBasicView && config.patchRedundancyRemoval) || 0 < config.maxEntityId;
    }
    return viewParamsList;
  }

  [[nodiscard]] static auto
  calculateViewGridSize(const Configuration &config,
                        const MivBitstream::ViewParamsList &viewParamsList) {
    int32_t x{};
    int32_t y{};

    const auto blockSize = config.blockSize;

    for (const auto &viewParams : viewParamsList) {
      x = std::max(x, (viewParams.ci.ci_projection_plane_width_minus1() + blockSize) / blockSize);
      y = std::max(y, (viewParams.ci.ci_projection_plane_height_minus1() + blockSize) / blockSize);
    }
    return Common::Vec2i{x, y};
  }

  [[nodiscard]] static auto
  calculateNominalAtlasFrameSizes(const Configuration &config,
                                  const MivBitstream::ViewParamsList &viewParamsList,
                                  double frameRate) {
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

    const auto blockSize = config.blockSize;
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
    const auto viewGridSize = calculateViewGridSize(config, viewParamsList);
    const auto atlasGridWidth = viewGridSize.x();
    const auto atlasGridHeight = maxBlocksPerAtlas2 / atlasGridWidth;

    // Warn if the aspect ratio is outside of HEVC limits (unlikely)
    if (atlasGridWidth * 8 < atlasGridHeight || atlasGridHeight * 8 < atlasGridWidth) {
      Common::logWarning("Atlas aspect ratio is outside of HEVC general tier and level limits");
    }

    return Common::SizeVector(numAtlases,
                              {atlasGridWidth * blockSize, atlasGridHeight * blockSize});
  }

  [[nodiscard]] static auto
  createGeometryInformation(uint32_t bitDepth, const MivBitstream::ViewParamsList &viewParamsList) {
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

  [[nodiscard]] static auto createOccupancyInformation(uint32_t bitDepth) {
    auto oi = MivBitstream::OccupancyInformation{};
    oi.oi_occupancy_codec_id(0)
        .oi_lossy_occupancy_compression_threshold(128)
        .oi_occupancy_2d_bit_depth_minus1(Common::downCast<uint8_t>(bitDepth - 1))
        .oi_occupancy_MSB_align_flag(false);
    return oi;
  }

  [[nodiscard]] static auto createAttributeInformation(uint32_t bitDepth) {
    auto ai = MivBitstream::AttributeInformation{};
    ai.ai_attribute_count(1)
        .ai_attribute_type_id(0, MivBitstream::AiAttributeTypeId::ATTR_TEXTURE)
        .ai_attribute_dimension_minus1(0, 2)
        .ai_attribute_2d_bit_depth_minus1(0, Common::downCast<uint8_t>(bitDepth - 1));
    return ai;
  }

  [[nodiscard]] static auto createGroupMapping(uint8_t numGroups,
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

  [[nodiscard]] static auto createVpsMiv2Extension(const Configuration &config,
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

  [[nodiscard]] static auto
  createV3cParameterSet(const Configuration &config,
                        const MivBitstream::ViewParamsList &viewParamsList, double frameRate) {
    const auto atlasFrameSizes = calculateNominalAtlasFrameSizes(config, viewParamsList, frameRate);

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

  [[nodiscard]] static auto log2FocLsbMinus4(int32_t intraPeriod) {
    // Avoid confusion but test MSB/LSB logic in decoder
    return Common::downCast<uint8_t>(std::max(4U, Common::ceilLog2(intraPeriod) + 1U) - 4U);
  }

  [[nodiscard]] static auto vuiParameters(const MivBitstream::SequenceConfig &sequenceConfig) {
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

  [[nodiscard]] static auto
  createCommonAtlasSequenceParameterSet(const Configuration &config,
                                        const MivBitstream::SequenceConfig &sequenceConfig,
                                        bool depthLowQualityFlag) {
    auto casps = MivBitstream::CommonAtlasSequenceParameterSetRBSP{};
    casps
        .casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4(
            log2FocLsbMinus4(config.intraPeriod))
        .casps_miv_extension()
        .casme_depth_low_quality_flag(depthLowQualityFlag)
        .casme_depth_quantization_params_present_flag(config.dqParamsPresentFlag)
        .vui_parameters(vuiParameters(sequenceConfig));
    return casps;
  }

  [[nodiscard]] static auto
  patchSizeQuantizers(const Configuration &config,
                      const MivBitstream::ViewParamsList &viewParamsList) {
    auto quantizer = config.blockSize;

    for (const auto &vp : viewParamsList) {
      quantizer = std::gcd(quantizer, vp.ci.ci_projection_plane_width_minus1() + 1);
      quantizer = std::gcd(quantizer, vp.ci.ci_projection_plane_height_minus1() + 1);
    }

    // NOTE(BK): There may be rotated patches of full width or height: same quantizer for x and y
    return Common::Vec2i{quantizer, quantizer};
  }

  [[nodiscard]] static auto createAspsMivExtension(const Configuration &config,
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

  [[nodiscard]] static auto createAtlasSequenceParameterSet(
      const Configuration &config, const MivBitstream::V3cParameterSet &vps,
      const MivBitstream::ViewParamsList &viewParamsList, MivBitstream::AtlasId j) {
    const auto blockSize = config.blockSize;
    const auto psq = patchSizeQuantizers(config, viewParamsList);

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
    asps.asps_miv_2_extension() = {};

    if (vps.vps_geometry_video_present_flag(j)) {
      const auto &gi = vps.geometry_information(j);
      asps.asps_geometry_3d_bit_depth_minus1(gi.gi_geometry_3d_coordinates_bit_depth_minus1())
          .asps_geometry_2d_bit_depth_minus1(gi.gi_geometry_2d_bit_depth_minus1());
    }
    return asps;
  }

  [[nodiscard]] static auto
  createAtlasTileHeader(const Configuration &config,
                        const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                        const MivBitstream::ViewParamsList &viewParamsList) {
    auto ath = MivBitstream::AtlasTileHeader{};
    ath.ath_type(MivBitstream::AthType::I_TILE)
        .ath_ref_atlas_frame_list_asps_flag(true)
        .ath_pos_min_d_quantizer(asps.asps_geometry_3d_bit_depth_minus1() + 1);

    if (asps.asps_patch_size_quantizer_present_flag()) {
      const auto psq = patchSizeQuantizers(config, viewParamsList);

      ath.ath_patch_size_x_info_quantizer(Common::ceilLog2(psq.x()));
      ath.ath_patch_size_y_info_quantizer(Common::ceilLog2(psq.y()));
    }
    return ath;
  }

  [[nodiscard]] static auto
  createEncoderAtlasParams(const Configuration &config, const MivBitstream::V3cParameterSet &vps,
                           const MivBitstream::ViewParamsList &viewParamsList,
                           MivBitstream::AtlasId j) {
    auto atlas = MivBitstream::EncoderAtlasParams{};
    atlas.asps = createAtlasSequenceParameterSet(config, vps, viewParamsList, j);
    atlas.athTemplate = createAtlasTileHeader(config, atlas.asps, viewParamsList);

    return atlas;
  }

  [[nodiscard]] static auto createEncoderParams(const Configuration &config,
                                                const MivBitstream::SequenceConfig &sequenceConfig,
                                                const MivBitstream::ViewParamsList &viewParamsList,
                                                bool depthLowQualityFlag) {
    auto params = EncoderParams{};

    params.viewParamsList = adaptViewParamsList(config, viewParamsList);

    params.vps = createV3cParameterSet(config, params.viewParamsList, sequenceConfig.frameRate);

    params.casps =
        createCommonAtlasSequenceParameterSet(config, sequenceConfig, depthLowQualityFlag);

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
                                                 params.vps.vps_atlas_id(k));
    }
    return params;
  }

  [[nodiscard]] static auto sampleBudget(const MivBitstream::V3cParameterSet &vps) {
    int32_t sampleBudget = 0;
    for (size_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto j = vps.vps_atlas_id(k);
      sampleBudget += (vps.vps_frame_width(j) * vps.vps_frame_height(j));
    }
    return sampleBudget;
  }

  void prepareSequence(const SourceUnit &unit) {
    m_transportViewParams = unit.viewParamsList;
    m_semiBasicViewCount = unit.semiBasicViewCount;

    m_params = createEncoderParams(m_config, unit.sequenceConfig, m_transportViewParams,
                                   unit.depthLowQualityFlag);

    const auto pruningParents = m_pruner->prepareSequence(
        {m_params.viewParamsList, unit.depthLowQualityFlag, sampleBudget(m_params.vps)});

    for (size_t viewIdx = 0; viewIdx < pruningParents.size(); ++viewIdx) {
      m_params.viewParamsList[viewIdx].pp = pruningParents[viewIdx];
    }
  }

  void prepareAccessUnit() {
    m_transportViews.clear();
    m_aggregatedEntityMask.clear();
    m_aggregator->prepareAccessUnit();
  }

  void pushSingleEntityFrame(Common::DeepFrameList transportViews) {
    const auto masks = m_pruner->prune(m_transportViewParams, transportViews, m_semiBasicViewCount);
    const auto informtaion = m_pruner->getPixelInformation();
    m_transportViews.push_back(std::move(transportViews));
    m_aggregator->pushMask(masks);
    m_aggregator->pushInformation(informtaion);
  }

  static auto entitySeparator(const Common::DeepFrameList &transportViews,
                              Common::SampleValue entityId) {
    Common::DeepFrameList entityViews;
    for (const auto &transportView : transportViews) {
      Common::DeepFrame entityView = {
          Common::Frame<>::yuv420(transportView.texture.getSize(),
                                  transportView.texture.getBitDepth()),
          Common::Frame<>::lumaOnly(transportView.geometry.getSize(),
                                    transportView.geometry.getBitDepth())};
      entityViews.push_back(std::move(entityView));
    }
    Common::FrameList<> entityMaps;
    for (const auto &transportView : transportViews) {
      entityMaps.push_back(transportView.entities);
    }

    auto entityMapsYUV = yuvSampler(entityMaps);

    for (size_t viewIdx = 0; viewIdx < transportViews.size(); viewIdx++) {
      const auto neutralColor = entityViews[viewIdx].texture.neutralValue();

      for (int32_t planeIdx = 0; planeIdx < 3; ++planeIdx) {
        std::transform(transportViews[viewIdx].texture.getPlane(planeIdx).begin(),
                       transportViews[viewIdx].texture.getPlane(planeIdx).end(),
                       entityMapsYUV[viewIdx].getPlane(planeIdx).begin(),
                       entityViews[viewIdx].texture.getPlane(planeIdx).begin(),
                       [=](auto i, auto j) { return (j == entityId) ? i : neutralColor; });
      }
      std::transform(transportViews[viewIdx].geometry.getPlane(0).begin(),
                     transportViews[viewIdx].geometry.getPlane(0).end(),
                     entityMaps[viewIdx].getPlane(0).begin(),
                     entityViews[viewIdx].geometry.getPlane(0).begin(),
                     [=](auto i, auto j) { return (j == entityId) ? i : uint16_t{}; });
    }

    return entityViews;
  }

  void pushMultiEntityFrame(Common::DeepFrameList transportViews) {
    Common::FrameList<uint8_t> mergedMasks;
    for (const auto &transportView : transportViews) {
      mergedMasks.emplace_back().createY(transportView.texture.getSize());
    }

    for (auto entityId = m_config.entityEncRange[0]; entityId < m_config.entityEncRange[1];
         entityId++) {
      Common::logInfo("Processing entity {}", entityId);

      const auto transportEntityViews = entitySeparator(transportViews, entityId);
      auto masks =
          m_pruner->prune(m_transportViewParams, transportEntityViews, m_semiBasicViewCount);
      updateMasks(transportEntityViews, masks);
      aggregateEntityMasks(masks, entityId);
      mergeMasks(mergedMasks, masks);
    }

    m_transportViews.push_back(std::move(transportViews));
    m_aggregator->pushMask(mergedMasks);
  }

  static auto yuvSampler(const Common::FrameList<> &in) -> Common::FrameList<> {
    Common::FrameList<> outYuvAll;

    for (const auto &entityMap : in) {
      auto outYuv = Common::Frame<>::yuv420(entityMap.getSize(), entityMap.getBitDepth());
      const auto width = entityMap.getWidth();
      const auto height = entityMap.getHeight();
      int32_t step = 1;
      for (int32_t k = 0; k < 3; ++k) {
        if (k != 0) {
          step = 2;
        }
        int32_t rowIdx = 0;
        for (int32_t i = 0; i != height; i = i + step) {
          int32_t colIdx = 0;
          for (int32_t j = 0; j != width; j = j + step) {
            outYuv.getPlane(k)(rowIdx, colIdx) = entityMap.getPlane(0)(i, j);
            colIdx++;
          }
          rowIdx++;
        }
      }
      outYuvAll.push_back(outYuv);
    }
    return outYuvAll;
  }

  static void mergeMasks(Common::FrameList<uint8_t> &mergedMasks,
                         Common::FrameList<uint8_t> masks) {
    for (size_t viewIdx = 0; viewIdx < mergedMasks.size(); viewIdx++) {
      for (size_t i = 0; i < mergedMasks[viewIdx].getPlane(0).size(); i++) {
        if (masks[viewIdx].getPlane(0)[i] != uint8_t{}) {
          mergedMasks[viewIdx].getPlane(0)[i] = masks[viewIdx].getPlane(0)[i];
        }
      }
    }
  }

  static void updateMasks(const Common::DeepFrameList &views, Common::FrameList<uint8_t> &masks) {
    for (size_t viewIdx = 0; viewIdx < views.size(); viewIdx++) {
      for (size_t i = 0; i < masks[viewIdx].getPlane(0).size(); i++) {
        if (views[viewIdx].geometry.getPlane(0)[i] == uint16_t{}) {
          masks[viewIdx].getPlane(0)[i] = uint8_t{};
        }
      }
    }
  }

  void aggregateEntityMasks(Common::FrameList<uint8_t> &masks, Common::SampleValue entityId) {
    if (m_aggregatedEntityMask.size() < m_config.entityEncRange[1] - m_config.entityEncRange[0]) {
      m_aggregatedEntityMask.push_back(masks);
    } else {
      for (size_t i = 0; i < masks.size(); i++) {
        auto &entityMaskPlane =
            m_aggregatedEntityMask[entityId - m_config.entityEncRange[0]][i].getPlane(0);
        std::transform(entityMaskPlane.begin(), entityMaskPlane.end(), masks[i].getPlane(0).begin(),
                       entityMaskPlane.begin(), [](auto v1, auto v2) { return std::max(v1, v2); });
      }
    }
  }

  void pushFrame(Common::DeepFrameList transportViews) {
    if (m_config.maxEntityId == 0) {
      pushSingleEntityFrame(std::move(transportViews));
    } else {
      pushMultiEntityFrame(std::move(transportViews));
    }
  }

  void pruningWithInformation(Common::FrameList<uint8_t> &aggregatedMask,
                              const Common::FrameList<uint32_t> &information) {
    std::vector<uint32_t> informationList;
    for (size_t i = 0; i < information.size(); i++) {
      for (size_t j = 0; j < information[i].getPlanes()[0].size(); j++) {
        if (aggregatedMask[i].getPlanes()[0][j] != 0) {
          informationList.push_back(information[i].getPlanes()[0][j]);
        }
      }
    }

    auto pixelReservedCnt = static_cast<int32_t>(informationList.size());
    int32_t pixelLimited = 0;
    for (const auto &item : m_params.atlas) {
      pixelLimited += item.asps.asps_frame_width() * item.asps.asps_frame_height();
    }

    if (pixelReservedCnt > pixelLimited) {
      Common::logInfo("Reserved pixel count over atlas limit {}M and pruning with Information",
                      1e-6 * static_cast<double>(pixelReservedCnt - pixelLimited));
      std::sort(informationList.rbegin(), informationList.rend());
      double information_threshold = informationList[pixelLimited];
      for (size_t i = 0; i < information.size(); i++) {
        for (size_t j = 0; j < information[i].getPlanes()[0].size(); j++) {
          if (aggregatedMask[i].getPlanes()[0][j] != 0) {
            if (information[i].getPlanes()[0][j] < information_threshold) {
              aggregatedMask[i].getPlanes()[0][j] = uint8_t{};
            }
          }
        }
      }
    }
  }

  auto setPartition() {
    PRECONDITION(!m_config.singleTileInAtlas());

    const auto sumPartitionWidth =
        std::accumulate(m_config.partitionWidth.begin(), m_config.partitionWidth.end(), int32_t{});
    const auto sumPartitionHeight = std::accumulate(m_config.partitionHeight.begin(),
                                                    m_config.partitionHeight.end(), int32_t{});

    for (const auto &atlas : m_params.atlas) {
      if (sumPartitionWidth != atlas.asps.asps_frame_width()) {
        Common::logWarning("the sum of tile width={}, the atlas width={}", sumPartitionWidth,
                           atlas.asps.asps_frame_width());
        throw std::runtime_error("Atlas width should be equal to sum of tile width");
      }
      if (sumPartitionHeight != atlas.asps.asps_frame_height()) {
        Common::logWarning("the sum of tile height={}, the atlas height={}", sumPartitionHeight,
                           atlas.asps.asps_frame_height());
        throw std::runtime_error("Atlas height should be equal to sum of tile height");
      }
    }

    const auto uniform =
        std::all_of(m_config.partitionWidth.begin(), m_config.partitionWidth.end(),
                    [&](int32_t value) { return value == m_config.partitionWidth.front(); }) &&
        std::all_of(m_config.partitionHeight.begin(), m_config.partitionHeight.end(),
                    [&](int32_t value) { return value == m_config.partitionHeight.front(); });

    std::vector<std::vector<int32_t>> partitionWidthList;
    std::vector<std::vector<int32_t>> partitionHeightList;
    std::vector<std::vector<int32_t>> partitionPosXList;
    std::vector<std::vector<int32_t>> partitionPosYList;

    for (size_t atlasIdx = 0; atlasIdx < m_params.atlas.size(); ++atlasIdx) {
      std::vector<int32_t> widthList;
      std::vector<int32_t> heightList;
      std::vector<int32_t> posXList;
      std::vector<int32_t> posYList;
      int32_t widthTemp = 0;
      int32_t heightTemp = 0;

      for (const auto &width : m_config.partitionWidth) {
        widthList.emplace_back(width);
        posXList.emplace_back(widthTemp);
        widthTemp += width;
      }
      for (const auto &height : m_config.partitionHeight) {
        heightList.emplace_back(height);
        posYList.emplace_back(heightTemp);
        heightTemp += height;
      }
      partitionHeightList.emplace_back(heightList);
      partitionPosXList.emplace_back(posXList);
      partitionPosYList.emplace_back(posYList);
      partitionWidthList.emplace_back(widthList);
    }

    m_partitionArray.emplace_back(partitionWidthList);
    m_partitionArray.emplace_back(partitionHeightList);
    m_partitionArray.emplace_back(partitionPosXList);
    m_partitionArray.emplace_back(partitionPosYList);

    return uniform;
  }

  void setTiles() {
    m_partitionArray.clear();

    for (auto &atlas : m_params.atlas) {
      atlas.tilePartitions.clear();
    }

    if (m_config.singleTileInAtlas()) {
      for (auto &atlas : m_params.atlas) {
        auto &tilePartition = atlas.tilePartitions.emplace_back();

        tilePartition.partitionPosX = 0;
        tilePartition.partitionPosY = 0;
        tilePartition.partitionHeight = atlas.asps.asps_frame_height();
        tilePartition.partitionWidth = atlas.asps.asps_frame_width();

        atlas.afps.atlas_frame_tile_information(MivBitstream::AtlasFrameTileInformation{}
                                                    .afti_single_tile_in_atlas_frame_flag(true)
                                                    .afti_num_tiles_in_atlas_frame_minus1(0));
      }
    } else {
      const auto uniformPartitionSpacingFlag = setPartition();

      for (size_t atlasIdx = 0; atlasIdx < m_params.atlas.size(); ++atlasIdx) {
        auto &atlas = m_params.atlas[atlasIdx];

        for (size_t i = 0; i < m_partitionArray[1][atlasIdx].size(); ++i) {
          for (size_t j = 0; j < m_partitionArray[0][atlasIdx].size(); ++j) {
            auto &tilePartition = atlas.tilePartitions.emplace_back();

            tilePartition.partitionWidth = m_partitionArray[0][atlasIdx][j];
            tilePartition.partitionHeight = m_partitionArray[1][atlasIdx][i];
            tilePartition.partitionPosX = m_partitionArray[2][atlasIdx][j];
            tilePartition.partitionPosY = m_partitionArray[3][atlasIdx][i];
          }
        }
      }

      setAtlasFrameTileInformation(uniformPartitionSpacingFlag);
    }
  }

  void setAtlasFrameTileInformation(bool uniformPartitionSpacingFlag) {
    for (size_t atlasIdx = 0; atlasIdx < m_params.atlas.size(); ++atlasIdx) {
      auto afti = TMIV::MivBitstream::AtlasFrameTileInformation{};

      afti.afti_single_tile_in_atlas_frame_flag(false);
      afti.afti_uniform_partition_spacing_flag(uniformPartitionSpacingFlag);
      afti.afti_single_partition_per_tile_flag(true);

      if (uniformPartitionSpacingFlag) {
        int32_t partitionColsWidthMinus1 = m_config.partitionWidth[0] / 64 - 1;
        VERIFY(m_config.partitionWidth[0] % 64 == 0);

        int32_t partitionRowsHeightMinus1 = m_config.partitionHeight[0] / 64 - 1;
        VERIFY(m_config.partitionHeight[0] % 64 == 0);

        afti.afti_partition_cols_width_minus1(partitionColsWidthMinus1)
            .afti_partition_rows_height_minus1(partitionRowsHeightMinus1);
      } else {
        afti.afti_num_partition_columns_minus1(
            Common::downCast<uint8_t>(m_config.partitionWidth.size() - 1));
        afti.afti_num_partition_rows_minus1(
            Common::downCast<uint8_t>(m_config.partitionHeight.size() - 1));

        for (uint8_t j = 0; j < afti.afti_num_partition_columns_minus1(); ++j) {
          if (m_partitionArray[0][atlasIdx][j] % 64 != 0) {
            TMIV::Common::logWarning("the width of the tile partition={}",
                                     m_partitionArray[0][atlasIdx][j]);
            throw std::runtime_error("the width of the tile partition should be in units of 64 "
                                     "samples accoding to 23090-5 clause 8.3.6.2.2");
          }
          afti.afti_partition_column_width_minus1(j, m_partitionArray[0][atlasIdx][j] / 64 - 1);
        }
        for (uint8_t j = 0; j < afti.afti_num_partition_rows_minus1(); ++j) {
          if (m_partitionArray[1][atlasIdx][j] % 64 != 0) {
            TMIV::Common::logWarning("the height of the tile partition={}",
                                     m_partitionArray[1][atlasIdx][j]);
            throw std::runtime_error("the height of the tile partition should be in units of 64 "
                                     "samples accoding to 23090-5 clause 8.3.6.2.2");
          }
          afti.afti_partition_row_height_minus1(j, m_partitionArray[1][atlasIdx][j] / 64 - 1);
        }
      }
      afti.afti_num_tiles_in_atlas_frame_minus1(Common::downCast<uint8_t>(
          afti.numPartitionsInAtlasFrame(m_params.atlas[atlasIdx].asps) - 1));

      m_params.atlas[atlasIdx].afps.atlas_frame_tile_information(afti);
    }
  }

  static void assignFullPatchRanges(EncoderParams &params) {
    for (auto &pp : params.patchParamsList) {
      const auto atlasIdx = params.vps.indexOf(pp.atlasId());
      const auto bitDepth = params.atlas[atlasIdx].asps.asps_geometry_2d_bit_depth_minus1() + 1U;
      pp.atlasPatch3dOffsetD(0).atlasPatch3dRangeD(Common::maxLevel(bitDepth));
    }
  }

  void completeAccessUnit() {
    Common::logVerbose("completeAccessUnit: FOC is {}.", m_params.foc);

    m_aggregator->completeAccessUnit();
    auto &aggregatedMask = m_aggregator->getAggregatedMask();
    auto &information = m_aggregator->getMeanAggregatedInformation();

    if (m_config.informationPruning && !information.empty()) {
      pruningWithInformation(aggregatedMask, information);
    }

    updateAggregationStatistics(aggregatedMask);

    if (0 < m_config.maxEntityId) {
      m_packer->updateAggregatedEntityMasks(m_aggregatedEntityMask);
    }

    setTiles();

    auto tilePartitions = std::vector<std::vector<MivBitstream::TilePartition>>{};
    auto tileSizes = std::vector<Common::SizeVector>{};

    for (const auto &atlas : m_params.atlas) {
      tilePartitions.push_back(atlas.tilePartitions);
      auto &sizes = tileSizes.emplace_back();

      for (const auto &tilePartition : atlas.tilePartitions) {
        sizes.push_back({tilePartition.partitionWidth, tilePartition.partitionHeight});
      }
    }

    m_packer->initialize(tilePartitions);
    m_packer->initialize(tileSizes, m_config.blockSize);
    m_params.patchParamsList = m_packer->pack(tileSizes, aggregatedMask, m_transportViewParams,
                                              m_config.blockSize, information);

    assignFullPatchRanges(m_params);
  }

  void updateAggregationStatistics(const Common::FrameList<uint8_t> &aggregatedMask) {
    const auto lumaSamplesPerFrame = std::accumulate(
        aggregatedMask.begin(), aggregatedMask.end(), size_t{}, [](size_t sum, const auto &mask) {
          return sum + 2 * std::count_if(mask.getPlane(0).begin(), mask.getPlane(0).end(),
                                         [](auto x) { return x > 0; });
        });
    Common::logInfo("Aggregated luma samples per frame is {}M",
                    1e-6 * static_cast<double>(lumaSamplesPerFrame));
    m_maxLumaSamplesPerFrame = std::max(m_maxLumaSamplesPerFrame, lumaSamplesPerFrame);
  }

  [[nodiscard]] auto calculateBtpm() const {
    std::vector<std::vector<std::vector<int32_t>>> btpm;
    for (uint8_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
      const auto &currentAtlas = m_videoFrameBuffer[0][k];
      int32_t AH = currentAtlas.texture.getHeight() / m_config.blockSize;
      int32_t AW = currentAtlas.texture.getWidth() / m_config.blockSize;
      std::vector<std::vector<int32_t>> tmphw;
      for (int32_t h = 0; h < AH; h++) {
        std::vector<int32_t> tmpw;
        tmpw.reserve(AW);
        for (int32_t w = 0; w < AW; w++) {
          tmpw.push_back(Common::unusedPatchIdx);
        }
        tmphw.push_back(tmpw);
      }
      btpm.push_back(tmphw);
    }
    return btpm;
  }

  void applyPatchTextureOffset() {
    std::vector<std::vector<std::vector<int32_t>>> btpm = calculateBtpm();

    adaptBtpmToPatchCount(btpm);

    const auto inputBitDepth = m_videoFrameBuffer.front().front().texture.getBitDepth();
    const auto bitShift =
        static_cast<int32_t>(inputBitDepth) - static_cast<int32_t>(m_config.texBitDepth);

    for (auto &videoFrame : m_videoFrameBuffer) {
      for (uint8_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
        auto &atlas = videoFrame[k];
        const auto &asme = m_params.atlas[k].asps.asps_miv_extension();
        auto occScaleX = 1;
        auto occScaleY = 1;

        if (!asme.asme_embedded_occupancy_enabled_flag() &&
            asme.asme_occupancy_scale_enabled_flag()) {
          occScaleX = asme.asme_occupancy_scale_factor_x_minus1() + 1;
          occScaleY = asme.asme_occupancy_scale_factor_y_minus1() + 1;
        }

        for (int32_t y = 0; y < atlas.texture.getHeight(); ++y) {
          for (int32_t x = 0; x < atlas.texture.getWidth(); ++x) {
            const auto patchIdx = btpm[k][y / m_config.blockSize][x / m_config.blockSize];

            if (patchIdx == Common::unusedPatchIdx ||
                (atlas.geometry.getPlane(0)(y, x) == 0 &&
                 !m_params
                      .viewParamsList[m_params.patchParamsList[patchIdx].atlasPatchProjectionId()]
                      .isBasicView)) {
              continue;
            }
            if (atlas.occupancy.getPlane(0)(y / occScaleY, x / occScaleX) == 0) {
              continue;
            }
            const auto &pp = m_params.patchParamsList[patchIdx];

            const auto applyOffset = [&](uint8_t c, int32_t i, int32_t j) {
              auto &sample = atlas.texture.getPlane(c)(i, j);
              sample = Common::assertDownCast<Common::DefaultElement>(
                  sample - Common::shift(pp.atlasPatchTextureOffset(c), bitShift));
            };

            applyOffset(0, y, x);

            if (y % 2 == 0 && x % 2 == 0) {
              applyOffset(1, y / 2, x / 2);
              applyOffset(2, y / 2, x / 2);
            }
          }
        }
      }
    }
  }

  void adaptBtpmToPatchCount(std::vector<std::vector<std::vector<int32_t>>> &btpm) const {
    int32_t patchCnt = 0;
    for (const auto &patch : m_params.patchParamsList) {
      size_t atlasId = m_params.vps.indexOf(patch.atlasId());

      const auto &currentAtlas = m_videoFrameBuffer[0][atlasId];
      int32_t AH = currentAtlas.texture.getHeight() / m_config.blockSize;
      int32_t AW = currentAtlas.texture.getWidth() / m_config.blockSize;

      int32_t w = patch.atlasPatch3dSizeU();
      int32_t h = patch.atlasPatch3dSizeV();
      int32_t xM = patch.atlasPatch3dOffsetU();
      int32_t yM = patch.atlasPatch3dOffsetV();

      for (int32_t dyAligned = 0; dyAligned < h; dyAligned += m_config.blockSize) {
        for (int32_t dxAligned = 0; dxAligned < w; dxAligned += m_config.blockSize) {
          for (int32_t dy = dyAligned; dy < dyAligned + m_config.blockSize; dy++) {
            for (int32_t dx = dxAligned; dx < dxAligned + m_config.blockSize; dx++) {
              Common::Vec2i pView = {xM + dx, yM + dy};
              Common::Vec2i pAtlas = patch.viewToAtlas(pView);

              int32_t ay = pAtlas.y() / m_config.blockSize;
              int32_t ax = pAtlas.x() / m_config.blockSize;

              if (ay < 0 || ax < 0 || ay >= AH || ax >= AW ||
                  pAtlas.y() % m_config.blockSize != 0 || pAtlas.x() % m_config.blockSize != 0) {
                continue;
              }

              btpm[atlasId][ay][ax] = patchCnt;
            }
          }
        }
      }

      patchCnt++;
    }
  }

  void encodePatchTextureOffset(const PatchTextureStats &stats) {
    const auto inputBitDepth = m_videoFrameBuffer.front().front().texture.getBitDepth();

    const auto muddle = [this, inputBitDepth]() {
      // Take into account three different bit depths
      const auto outputBitDepth = m_config.texBitDepth;
      const auto offsetBitDepth = m_config.textureOffsetBitCount;
      const auto minBitDepth = std::min({inputBitDepth, outputBitDepth, offsetBitDepth});

      // Implement round-towards-zero behaviour
      static_assert((-13 >> 2) == -4); // bad
      static_assert((-13 / 4) == -3);  // good
      const auto divisor = int64_t{1} << (inputBitDepth - minBitDepth);
      const auto scaler = int64_t{1} << (outputBitDepth - minBitDepth);

      return [=](int64_t x) { return Common::downCast<int32_t>((x / divisor) * scaler); };
    }();

    const int64_t m = Common::medLevel(inputBitDepth);
    const int64_t L = Common::maxLevel(inputBitDepth);

    // Constrained optimization to find the atlas patch texture offset o, at input bit depth:
    //
    // input        :       0 <= s.min()     <= s.mean()     <= s.max()     <= L
    // output       :       0 <= s.min() - o <= s.mean() - o <= s.max() - o <= L
    // target       : s.mean(s) - o == m            =>            o == mean(s) - m
    // constraint 1 :             0 <= s.min() - o  =>            o <= s.min()
    // constraint 2 :   s.max() - o <= L            =>  s.max() - L <= o

    for (size_t p = 0; p != m_params.patchParamsList.size(); ++p) {
      for (uint8_t c = 0; c < 3; ++c) {
        const auto &s = stats[p][c];
        m_params.patchParamsList[p].atlasPatchTextureOffset(
            c, muddle(std::clamp(s.floorMean() - m, s.max() - L, s.min())));
      }
    }
  }

  void scaleChromaDynamicRange() {
    auto viewTextureStats = PatchTextureStats(m_params.viewParamsList.size());
    const auto numOfFrames = m_transportViews.size();
    const auto numOfViews = m_transportViews[0].size();
    const auto maxValue = (1 << m_transportViews[0].front().texture.getBitDepth()) - 1;

    for (size_t f = 0; f < numOfFrames; f++) {
      for (size_t v = 0; v < numOfViews; v++) {
        auto &frame = m_transportViews[f][v].texture;
        const auto W = frame.getWidth() / 2;
        const auto H = frame.getHeight() / 2;

        for (int32_t c = 1; c < 3; ++c) {
          for (int32_t h = 0; h < H; h++) {
            for (int32_t w = 0; w < W; w++) {
              const auto sample = frame.getPlane(c)(h, w);
              viewTextureStats[v][c] << sample;
            }
          }
        }
      }
    }

    for (size_t v = 0; v < numOfViews; v++) {
      m_params.viewParamsList[v].cs.cs_u_min(static_cast<uint16_t>(viewTextureStats[v][1].min()));
      m_params.viewParamsList[v].cs.cs_u_max(static_cast<uint16_t>(viewTextureStats[v][1].max()));
      m_params.viewParamsList[v].cs.cs_v_min(static_cast<uint16_t>(viewTextureStats[v][2].min()));
      m_params.viewParamsList[v].cs.cs_v_max(static_cast<uint16_t>(viewTextureStats[v][2].max()));
    }

    for (size_t f = 0; f < numOfFrames; f++) {
      for (size_t v = 0; v < numOfViews; v++) {
        auto &frame = m_transportViews[f][v].texture;
        const auto W = frame.getWidth() / 2;
        const auto H = frame.getHeight() / 2;

        for (int32_t c = 1; c < 3; ++c) {
          for (int32_t h = 0; h < H; h++) {
            for (int32_t w = 0; w < W; w++) {
              const auto sample = frame.getPlane(c)(h, w);
              frame.getPlane(c)(h, w) = static_cast<uint16_t>(
                  (static_cast<int64_t>(sample) - viewTextureStats[v][c].min()) * maxValue /
                  (viewTextureStats[v][c].max() - viewTextureStats[v][c].min()));
            }
          }
        }
      }
    }
  }

  [[nodiscard]] auto allocateAtlasList(uint32_t texBitDepth) const {
    auto atlasList = Common::DeepFrameList{};

    const auto &vps = m_params.vps;

    for (size_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      auto &frame = atlasList.emplace_back();
      const auto j = vps.vps_atlas_id(k);
      const auto frameWidth = vps.vps_frame_width(j);
      const auto frameHeight = vps.vps_frame_height(j);

      frame.patchIdx.createY({frameWidth, frameHeight});
      frame.patchIdx.fillValue(Common::unusedPatchIdx);

      if (m_config.haveTexture) {
        frame.texture.createYuv420({frameWidth, frameHeight}, texBitDepth);
        frame.texture.fillNeutral();
      }

      if (m_config.haveGeometry) {
        frame.geometry.createY({frameWidth, frameHeight});
        frame.geometry.fillZero();
      }

      const auto &asme = m_params.atlas[k].asps.asps_miv_extension();

      int32_t occFrameWidth = frameWidth / (asme.asme_occupancy_scale_factor_x_minus1() + 1);
      int32_t occFrameHeight = frameHeight / (asme.asme_occupancy_scale_factor_y_minus1() + 1);

      frame.occupancy.createY({occFrameWidth, occFrameHeight}, 1);
      frame.occupancy.fillZero();
    }
    return atlasList;
  }

  [[nodiscard]] auto isRedundantBlock(Common::Vec2i topLeft, Common::Vec2i bottomRight,
                                      uint16_t viewIdx) const {
    if (!m_config.patchRedundancyRemoval) {
      return false;
    }
    bottomRight.x() = std::min(topLeft.x() + m_config.blockSize, bottomRight.x());
    bottomRight.y() = std::min(topLeft.y() + m_config.blockSize, bottomRight.y());

    for (int32_t y = topLeft.y(); y < bottomRight.y(); ++y) {
      for (int32_t x = topLeft.x(); x < bottomRight.x(); ++x) {
        if (m_aggregator->getAggregatedMask()[viewIdx].getPlane(0)(y, x) != 0U) {
          return false;
        }
      }
    }
    return true;
  }

  template <typename Invocable>
  void visitPatch(const MivBitstream::PatchParams &patchParams, Invocable &&visit) const {
    const auto sizeU = patchParams.atlasPatch3dSizeU();
    const auto sizeV = patchParams.atlasPatch3dSizeV();
    const auto posU = patchParams.atlasPatch3dOffsetU();
    const auto posV = patchParams.atlasPatch3dOffsetV();

    for (int32_t vBlock = 0; vBlock < sizeV; vBlock += m_config.blockSize) {
      for (int32_t uBlock = 0; uBlock < sizeU; uBlock += m_config.blockSize) {
        const auto viewIdx = m_params.viewParamsList.indexOf(patchParams.atlasPatchProjectionId());
        const auto redundant =
            m_config.haveGeometry &&
            isRedundantBlock({posU + uBlock, posV + vBlock}, {posU + sizeU, posV + sizeV}, viewIdx);

        for (int32_t v = vBlock; v < vBlock + m_config.blockSize && v < sizeV; ++v) {
          for (int32_t u = uBlock; u < uBlock + m_config.blockSize && u < sizeU; ++u) {
            const auto pView = Common::Vec2i{posU + u, posV + v};
            const auto pAtlas = patchParams.viewToAtlas(pView);

            visit(pView, pAtlas, redundant);
          }
        }
      }
    }
  }

  auto writeSampleInPatchIdxMap(const MivBitstream::PatchParams &patchParams,
                                Common::DeepFrameList &frame, size_t patchIdx) const {
    const auto k = m_params.vps.indexOf(patchParams.atlasId());
    auto &atlas = frame[k];

    return [&, patchIdx](Common::Vec2i /* pView */, Common::Vec2i pAtlas, bool /* redundant */) {
      atlas.patchIdx.getPlane(0)(pAtlas.y(), pAtlas.x()) =
          Common::downCast<Common::DefaultElement>(patchIdx);
    };
  }

  auto writeOccupancySampleInAtlas(const MivBitstream::PatchParams &patchParams,
                                   const Common::DeepFrame &view,
                                   Common::DeepFrameList &frame) const {
    const auto k = m_params.vps.indexOf(patchParams.atlasId());
    auto &atlas = frame[k];

    const auto &inViewParams = m_transportViewParams[patchParams.atlasPatchProjectionId()];
    const auto &outViewParams = m_params.viewParamsList[patchParams.atlasPatchProjectionId()];

    return [&, this, k](Common::Vec2i pView, Common::Vec2i pAtlas, bool redundant) {
      const auto &asme = m_params.atlas[k].asps.asps_miv_extension();

      const auto yOcc = pAtlas.y() / (asme.asme_occupancy_scale_factor_y_minus1() + 1);
      const auto xOcc = pAtlas.x() / (asme.asme_occupancy_scale_factor_x_minus1() + 1);

      if (redundant) {
        if (m_config.haveOccupancy) {
          atlas.occupancy.getPlane(0)(yOcc, xOcc) = 0;
        }
      } else {
        auto depth = view.geometry.getPlane(0)(pView.y(), pView.x());

        atlas.occupancy.getPlane(0)(yOcc, xOcc) = 1;

        if (depth == 0 && !inViewParams.hasOccupancy && outViewParams.hasOccupancy &&
            asme.asme_max_entity_id() == 0) {
          depth = 1; // Avoid marking valid depth as invalid
        }

        if (depth == 0 && inViewParams.hasOccupancy) {
          atlas.occupancy.getPlane(0)(yOcc, xOcc) = 0;
        }

        if (depth > 0 && m_config.haveOccupancy) {
          atlas.occupancy.getPlane(0)(yOcc, xOcc) = 1;
        }
      }
    };
  }

  auto writeGeometrySampleInAtlas(const MivBitstream::PatchParams &patchParams,
                                  const Common::DeepFrame &view,
                                  Common::DeepFrameList &frame) const {
    const auto k = m_params.vps.indexOf(patchParams.atlasId());
    auto &atlas = frame[k];

    const auto &inViewParams = m_transportViewParams[patchParams.atlasPatchProjectionId()];
    const auto &outViewParams = m_params.viewParamsList[patchParams.atlasPatchProjectionId()];

    return [&, this, k](Common::Vec2i pView, Common::Vec2i pAtlas, bool redundant) {
      if (redundant) {
        atlas.geometry.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
      } else {
        auto depth = view.geometry.getPlane(0)(pView.y(), pView.x());

        if (depth == 0 && !inViewParams.hasOccupancy && outViewParams.hasOccupancy &&
            m_params.atlas[k].asps.asps_miv_extension().asme_max_entity_id() == 0) {
          depth = 1; // Avoid marking valid depth as invalid
        }

        atlas.geometry.getPlane(0)(pAtlas.y(), pAtlas.x()) = depth;
      }
    };
  }

  auto writeTextureSampleInAtlas(const MivBitstream::PatchParams &patchParams,
                                 const Common::DeepFrame &view,
                                 Common::DeepFrameList &frame) const {
    const auto k = m_params.vps.indexOf(patchParams.atlasId());
    auto &atlas = frame[k];

    return [&](Common::Vec2i pView, Common::Vec2i pAtlas, bool redundant) {
      if (redundant) {
        const auto textureMedVal = Common::medLevel<uint16_t>(atlas.texture.getBitDepth());

        atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x()) = textureMedVal;
        if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
          atlas.texture.getPlane(1)(pAtlas.y() / 2, pAtlas.x() / 2) = textureMedVal;
          atlas.texture.getPlane(2)(pAtlas.y() / 2, pAtlas.x() / 2) = textureMedVal;
        }
      } else {
        for (int32_t c = 0; c < 3; ++c) {
          const auto n = 0 < c ? 2 : 1;

          if (pView.x() % n == 0 && pView.y() % n == 0) {
            atlas.texture.getPlane(c)(pAtlas.y() / n, pAtlas.x() / n) =
                view.texture.getPlane(c)(pView.y() / n, pView.x() / n);
          }
        }
      }
    };
  }

  void writePatchInAtlas(const MivBitstream::PatchParams &patchParams,
                         const Common::DeepFrame &view, Common::DeepFrameList &frame,
                         size_t patchIdx) {
    visitPatch(patchParams, writeSampleInPatchIdxMap(patchParams, frame, patchIdx));

    if (m_config.haveGeometry) {
      visitPatch(patchParams, writeOccupancySampleInAtlas(patchParams, view, frame));
      visitPatch(patchParams, writeGeometrySampleInAtlas(patchParams, view, frame));
    }

    if (m_config.haveTexture) {
      visitPatch(patchParams, writeTextureSampleInAtlas(patchParams, view, frame));
    }
  }

  auto collectPatchTextureStats(const MivBitstream::PatchParams &patchParams,
                                const Common::DeepFrameList &frame) {
    PRECONDITION(m_config.haveTexture);

    auto textureStats = TextureStats{};

    const auto k = m_params.vps.indexOf(patchParams.atlasId());
    const auto &atlas = frame[k];

    visitPatch(patchParams, [&](Common::Vec2i /* pView */, Common::Vec2i pAtlas, bool redundant) {
      if (!redundant) {
        for (int32_t c = 0; c < 3; ++c) {
          const auto n = 0 < c ? 2 : 1;

          if (pAtlas.x() % n == 0 && pAtlas.y() % n == 0) {
            textureStats[c] << atlas.texture.getPlane(c)(pAtlas.y() / n, pAtlas.x() / n);
          }
        }
      }
    });

    return textureStats;
  }

  void constructVideoFrames() {
    auto patchTextureStats = PatchTextureStats(m_params.patchParamsList.size());

    for (const auto &views : m_transportViews) {
      auto atlasList = allocateAtlasList(views.front().texture.getBitDepth());

      for (size_t p = 0; p < m_params.patchParamsList.size(); p++) {
        const auto &patch = m_params.patchParamsList[p];
        const auto viewIdx = m_params.viewParamsList.indexOf(patch.atlasPatchProjectionId());
        const auto &view = views[viewIdx];
        const auto k = m_params.vps.indexOf(patch.atlasId());

        if (0 < m_params.atlas[k].asps.asps_miv_extension().asme_max_entity_id()) {
          Common::DeepFrameList tempViews;
          tempViews.push_back(view);
          const auto &entityViews = entitySeparator(tempViews, patch.atlasPatchEntityId());
          writePatchInAtlas(patch, entityViews[0], atlasList, p);
        } else {
          writePatchInAtlas(patch, view, atlasList, p);
        }

        if (m_config.haveTexture) {
          patchTextureStats[p] += collectPatchTextureStats(patch, atlasList);
        }
      }
      m_videoFrameBuffer.push_back(std::move(atlasList));
    }

    if (m_config.textureOffsetFlag) {
      encodePatchTextureOffset(patchTextureStats);
      applyPatchTextureOffset();
    }
  }

  std::vector<std::vector<std::vector<int32_t>>> m_partitionArray;

  // Encoder sub-components
  std::unique_ptr<Pruner::IPruner> m_pruner;
  std::unique_ptr<Aggregator::IAggregator> m_aggregator;
  std::unique_ptr<Packer::IPacker> m_packer;

  Configuration m_config;

  // View-optimized encoder input
  MivBitstream::ViewParamsList m_transportViewParams;
  int32_t m_semiBasicViewCount{};
  std::vector<Common::DeepFrameList> m_transportViews;

  EncoderParams m_params;
  int32_t m_firstIdx{};
  int32_t m_lastIdx{};

  std::vector<Common::DeepFrameList> m_videoFrameBuffer;

  // Mask aggregation state
  std::vector<Common::FrameList<uint8_t>> m_aggregatedEntityMask;
  size_t m_maxLumaSamplesPerFrame{};
};

Encoder::Encoder(const Common::Json &componentNode) : m_impl{new Impl{componentNode}} {}

Encoder::~Encoder() = default;

auto Encoder::maxLumaSamplesPerFrame() const -> size_t { return m_impl->maxLumaSamplesPerFrame(); }

auto Encoder::isStart(const SourceUnit &unit) -> bool { return m_impl->isStart(unit); }

void Encoder::process(std::vector<SourceUnit> buffer) {
  Common::logDebug("Encoder stage, processing {} source units", buffer.size());

  m_impl->process(std::move(buffer), source);
}
} // namespace TMIV::Encoder
