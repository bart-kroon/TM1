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

#include <TMIV/Encoder/Encoder.h>

#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/SequenceConfig.h>

#include <iostream>
#include <numeric>

namespace TMIV::Encoder {
void Encoder::prepareSequence(const MivBitstream::SequenceConfig &sequenceConfig,
                              const Common::MVD16Frame &firstFrame) {
  const auto depthLowQualityFlag = [&]() {
    if (m_config.depthLowQualityFlag) {
      return *m_config.depthLowQualityFlag;
    }
    if (m_config.haveGeometry) {
      return m_depthQualityAssessor->isLowDepthQuality(sequenceConfig.sourceViewParams(),
                                                       firstFrame);
    }
    return false;
  }();

  m_config.blockSize = m_config.blockSizeDepthQualityDependent[depthLowQualityFlag ? 1 : 0];
  VERIFY(2 <= m_config.blockSize);
  VERIFY((m_config.blockSize & (m_config.blockSize - 1)) == 0);

  const auto lumaSamplesPerAtlasSample =
      (m_config.haveTexture ? 1. : 0.) +
      (m_config.haveGeometry ? (m_config.geometryScaleEnabledFlag ? 0.25 : 1.) : 0.);
  const auto numGroups = std::max(1.F, static_cast<float>(m_config.numGroups));

  m_config.maxBlockRate = m_config.maxLumaSampleRate /
                          (numGroups * lumaSamplesPerAtlasSample * Common::sqr(m_config.blockSize));
  m_config.maxBlocksPerAtlas = m_config.maxLumaPictureSize / Common::sqr(m_config.blockSize);

  // Transform source to transport view sequence parameters
  m_transportParams =
      m_viewOptimizer->optimizeParams({sequenceConfig.sourceViewParams(), depthLowQualityFlag});

  // Calculate nominal atlas frame sizes
  const auto atlasFrameSizes =
      calculateNominalAtlasFrameSizes(m_transportParams.viewParamsList, sequenceConfig.frameRate);
  std::cout << "Nominal atlas frame sizes: { ";
  for (const auto &size : atlasFrameSizes) {
    std::cout << ' ' << size;
  }
  std::cout << " }\n";

  // Create IVS with VPS with right number of atlases but copy other parts from input IVS
  m_params = {};

  m_params.vps = createVps(atlasFrameSizes);
  m_params.viewParamsList = m_transportParams.viewParamsList;
  m_params.frameRate = sequenceConfig.frameRate;
  m_params.lengthsInMeters = sequenceConfig.lengthsInMeters;
  m_params.maxEntityId = m_config.maxEntityId;
  m_params.casps.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4())
      .casps_miv_extension()
      .casme_depth_low_quality_flag(depthLowQualityFlag)
      .casme_depth_quantization_params_present_flag(m_config.dqParamsPresentFlag)
      .vui_parameters(vuiParameters());

  m_params.viewingSpace = m_config.viewingSpace;

  if (m_config.viewportCameraParametersSei) {
    m_params.viewportCameraParameters = MivBitstream::ViewportCameraParameters::fromViewParams(
        sequenceConfig.cameraByName("viewport").viewParams);
  }
  if (m_config.viewportPositionSei) {
    m_params.viewportPosition = MivBitstream::ViewportPosition::fromViewParams(
        sequenceConfig.cameraByName("viewport").viewParams);
  }
  m_params.randomAccess = m_config.randomAccess;

  setGiGeometry3dCoordinatesBitdepthMinus1();

  int32_t sampleBudget = 0;
  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_params.vps.vps_atlas_id(k);
    sampleBudget += (m_params.vps.vps_frame_width(j) * m_params.vps.vps_frame_height(j));
  }

  // Register pruning relation
  m_params.viewParamsList = m_pruner->prepareSequence(Pruner::PrunerParams{
      std::move(m_params.viewParamsList),
      m_params.casps.casps_miv_extension().casme_depth_low_quality_flag(), sampleBudget});

  // Turn on occupancy coding per view
  enableOccupancyPerView();

  // Set-up ASPS and AFPS
  prepareIvau();
}

// Calculate atlas frame sizes [MPEG/M52994 v2]
auto Encoder::calculateNominalAtlasFrameSizes(const MivBitstream::ViewParamsList &viewParamsList,
                                              double frameRate) const -> Common::SizeVector {
  if (m_config.oneViewPerAtlasFlag) {
    // No constraints: one atlas per transport view
    auto result = Common::SizeVector(viewParamsList.size());
    std::transform(std::cbegin(viewParamsList), std::cend(viewParamsList), std::begin(result),
                   [](const MivBitstream::ViewParams &x) { return x.ci.projectionPlaneSize(); });
    return result;
  }

  if (!m_config.overrideAtlasFrameSizes.empty()) {
    std::cout
        << "WARNING: When overriding nominal atlas frame sizes, constraints are not checked.\n";
    return m_config.overrideAtlasFrameSizes;
  }

  // Translate block rate into a maximum number of blocks
  const auto maxBlocks = static_cast<int>(m_config.maxBlockRate / frameRate);

  // Calculate the number of atlases
  auto numAtlases = (maxBlocks + m_config.maxBlocksPerAtlas - 1) / m_config.maxBlocksPerAtlas;
  if (numAtlases > m_config.maxAtlases) {
    std::cout << "The maxAtlases constraint is a limiting factor.\n";
    numAtlases = m_config.maxAtlases;
  }

  // Calculate the number of blocks per atlas
  auto maxBlocksPerAtlas = maxBlocks / numAtlases;
  if (maxBlocksPerAtlas > m_config.maxBlocksPerAtlas) {
    std::cout << "The maxLumaPictureSize constraint is a limiting factor.\n";
    maxBlocksPerAtlas = m_config.maxBlocksPerAtlas;
  }

  // Take the smallest reasonable width
  const auto viewGridSize = calculateViewGridSize(viewParamsList);
  const auto atlasGridWidth = viewGridSize.x();
  const auto atlasGridHeight = maxBlocksPerAtlas / atlasGridWidth;

  // Warn if the aspect ratio is outside of HEVC limits (unlikely)
  if (atlasGridWidth * 8 < atlasGridHeight || atlasGridHeight * 8 < atlasGridWidth) {
    std::cout << "WARNING: Atlas aspect ratio is outside of HEVC general tier and level limits\n";
  }

  return Common::SizeVector(
      numAtlases, {atlasGridWidth * m_config.blockSize, atlasGridHeight * m_config.blockSize});
}

auto Encoder::calculateViewGridSize(const MivBitstream::ViewParamsList &viewParamsList) const
    -> Common::Vec2i {
  int x{};
  int y{};

  for (const auto &viewParams : viewParamsList) {
    x = std::max(x, (viewParams.ci.ci_projection_plane_width_minus1() + m_config.blockSize) /
                        m_config.blockSize);
    y = std::max(y, (viewParams.ci.ci_projection_plane_height_minus1() + m_config.blockSize) /
                        m_config.blockSize);
  }

  return {x, y};
}

auto Encoder::createVps(const std::vector<Common::Vec2i> &atlasFrameSizes) const
    -> MivBitstream::V3cParameterSet {
  auto vps = MivBitstream::V3cParameterSet{};

  vps.profile_tier_level()
      .ptl_level_idc(MivBitstream::PtlLevelIdc::Level_3_5)
      .ptl_profile_codec_group_idc(m_config.codecGroupIdc)
      .ptl_profile_reconstruction_idc(MivBitstream::PtlProfileReconstructionIdc::MIV_Main)
      .ptl_profile_toolset_idc(m_config.toolsetIdc);

  VERIFY_MIVBITSTREAM(!atlasFrameSizes.empty());
  vps.vps_atlas_count_minus1(static_cast<uint8_t>(atlasFrameSizes.size() - 1));

  for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
    const auto j = MivBitstream::AtlasId{k};
    vps.vps_atlas_id(k, j)
        .vps_frame_width(j, atlasFrameSizes[k].x())
        .vps_frame_height(j, atlasFrameSizes[k].y())
        .vps_geometry_video_present_flag(j, m_config.haveGeometry)
        .vps_occupancy_video_present_flag(j, m_config.haveOccupancy)
        .vps_attribute_video_present_flag(j, m_config.haveTexture);

    if (m_config.haveGeometry) {
      vps.geometry_information(j).gi_geometry_2d_bit_depth_minus1(9);
    }

    if (m_config.haveOccupancy) {
      vps.occupancy_information(j)
          .oi_occupancy_codec_id(0)
          .oi_lossy_occupancy_compression_threshold(128)
          .oi_occupancy_2d_bit_depth_minus1(9)
          .oi_occupancy_MSB_align_flag(false);
    }

    if (m_config.haveTexture) {
      vps.attribute_information(j)
          .ai_attribute_count(1)
          .ai_attribute_type_id(0, MivBitstream::AiAttributeTypeId::ATTR_TEXTURE)
          .ai_attribute_dimension_minus1(0, 2)
          .ai_attribute_2d_bit_depth_minus1(0, 9);
    }
  }

  auto &vme = vps.vps_miv_extension();
  vme.group_mapping().gm_group_count(m_config.numGroups);

  if (0 < vme.group_mapping().gm_group_count()) {
    // Group atlases together to restrict atlas-level sub-bitstream access
    for (size_t i = 0; i < atlasFrameSizes.size(); ++i) {
      vme.group_mapping().gm_group_id(i, 0);
    }
  }
  return vps;
}

auto Encoder::vuiParameters() const -> MivBitstream::VuiParameters {
  auto numUnitsInTick = 1;
  auto timeScale = static_cast<int>(numUnitsInTick * m_params.frameRate);
  LIMITATION(timeScale == numUnitsInTick * m_params.frameRate);

  auto vui = MivBitstream::VuiParameters{};
  vui.vui_num_units_in_tick(numUnitsInTick)
      .vui_time_scale(timeScale)
      .vui_poc_proportional_to_timing_flag(false)
      .vui_hrd_parameters_present_flag(false);
  vui.vui_unit_in_metres_flag(m_params.lengthsInMeters);
  vui.coordinate_system_parameters() = {};
  return vui;
}

void Encoder::setGiGeometry3dCoordinatesBitdepthMinus1() {
  uint8_t numBitsMinus1 = 9; // Main 10
  for (auto &vp : m_params.viewParamsList) {
    const auto size = std::max(vp.ci.ci_projection_plane_width_minus1() + 1,
                               vp.ci.ci_projection_plane_height_minus1() + 1);
    numBitsMinus1 = std::max(numBitsMinus1, static_cast<uint8_t>(Common::ceilLog2(size) - 1));
  }
  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_params.vps.vps_atlas_id(k);
    if (m_params.vps.vps_geometry_video_present_flag(j)) {
      m_params.vps.geometry_information(j).gi_geometry_3d_coordinates_bit_depth_minus1(
          numBitsMinus1);
    }
  }
}

void Encoder::enableOccupancyPerView() {
  for (size_t viewIdx = 0; viewIdx < m_params.viewParamsList.size(); ++viewIdx) {
    if (!m_params.viewParamsList[viewIdx].isBasicView || 0 < m_params.maxEntityId) {
      m_params.viewParamsList[viewIdx].hasOccupancy = true;
    }
  }
}

void Encoder::prepareIvau() {
  m_params.atlas.resize(m_params.vps.vps_atlas_count_minus1() + size_t{1});

  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    auto &atlas = m_params.atlas[k];
    const auto j = m_params.vps.vps_atlas_id(k);

    // Set ASPS parameters
    atlas.asps.asps_frame_width(m_params.vps.vps_frame_width(j))
        .asps_frame_height(m_params.vps.vps_frame_height(j))
        .asps_log2_max_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4())
        .asps_use_eight_orientations_flag(true)
        .asps_extended_projection_enabled_flag(true)
        .asps_normal_axis_limits_quantization_enabled_flag(true)
        .asps_max_number_projections_minus1(
            static_cast<uint16_t>(m_params.viewParamsList.size() - 1))
        .asps_log2_patch_packing_block_size(Common::ceilLog2(m_config.blockSize))
        .asps_num_ref_atlas_frame_lists_in_asps(1);

    const auto psq = patchSizeQuantizers();
    atlas.asps.asps_patch_size_quantizer_present_flag(psq.x() != m_config.blockSize ||
                                                      psq.y() != m_config.blockSize);

    if (m_params.vps.vps_geometry_video_present_flag(j)) {
      const auto &gi = m_params.vps.geometry_information(j);
      atlas.asps.asps_geometry_3d_bit_depth_minus1(gi.gi_geometry_3d_coordinates_bit_depth_minus1())
          .asps_geometry_2d_bit_depth_minus1(gi.gi_geometry_2d_bit_depth_minus1());
    }

    if (0 < m_params.maxEntityId) {
      atlas.asps.asps_miv_extension().asme_max_entity_id(m_params.maxEntityId);
    }

    // Set ATH parameters
    atlas.ath.ath_type(MivBitstream::AthType::I_TILE).ath_ref_atlas_frame_list_asps_flag(true);

    if (atlas.asps.asps_patch_size_quantizer_present_flag()) {
      atlas.ath.ath_patch_size_x_info_quantizer(Common::ceilLog2(psq.x()));
      atlas.ath.ath_patch_size_y_info_quantizer(Common::ceilLog2(psq.y()));
    }
    atlas.ath.ath_pos_min_d_quantizer( // make pdu_3d_offset_d u(0)
        atlas.asps.asps_geometry_3d_bit_depth_minus1() + 1);
  }
}

auto Encoder::log2FocLsbMinus4() const -> uint8_t {
  // Avoid confusion but test MSB/LSB logic in decoder
  return Common::downCast<uint8_t>(std::max(4U, Common::ceilLog2(m_config.intraPeriod) + 1U) - 4U);
}

auto Encoder::patchSizeQuantizers() const -> Common::Vec2i {
  auto quantizer = m_config.blockSize;

  for (const auto &vp : m_params.viewParamsList) {
    quantizer = std::gcd(quantizer, vp.ci.ci_projection_plane_width_minus1() + 1);
    quantizer = std::gcd(quantizer, vp.ci.ci_projection_plane_height_minus1() + 1);
  }

  // NOTE(BK): There may be rotated patches of full width or height: same quantizer for x and y
  return {quantizer, quantizer};
}
} // namespace TMIV::Encoder
