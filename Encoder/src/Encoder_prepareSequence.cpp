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

#include <TMIV/Encoder/Encoder.h>

#include <TMIV/MivBitstream/verify.h>

#include <cassert>
#include <iostream>

namespace TMIV::Encoder {
namespace {
void runtimeCheck(bool cond, const char *what) {
  if (!cond) {
    throw std::runtime_error(what);
  }
}
} // namespace

void Encoder::prepareSequence(MivBitstream::EncoderParams sourceParams) {
  m_blockSize = m_blockSizeDepthQualityDependent[static_cast<std::size_t>(sourceParams.vme().vme_depth_low_quality_flag())];
  runtimeCheck(2 <= m_blockSize, "blockSize should be at least two");
  runtimeCheck((m_blockSize & (m_blockSize - 1)) == 0, "blockSize should be a power of two");

  const auto lumaSamplesPerAtlasSample = m_geometryScaleEnabledFlag ? 1.25 : 2.;
  m_maxBlockRate = m_maxLumaSampleRate / ((sourceParams.vme().vme_num_groups_minus1() + 1.) *
                                          lumaSamplesPerAtlasSample * Common::sqr(m_blockSize));
  m_maxBlocksPerAtlas = m_maxLumaPictureSize / Common::sqr(m_blockSize);

  // Transform source to transport view sequence parameters
  m_transportParams = m_viewOptimizer->optimizeParams(std::move(sourceParams));

  // Calculate nominal atlas frame sizes
  const auto atlasFrameSizes = calculateNominalAtlasFrameSizes(m_transportParams);
  std::cout << "Nominal atlas frame sizes: { ";
  for (auto &size : atlasFrameSizes) {
    std::cout << ' ' << size;
  }
  std::cout << " }\n";

  // Create IVS with VPS with right number of atlases but copy other parts from input IVS
  m_params = MivBitstream::EncoderParams{atlasFrameSizes, haveTexture(), haveOccupancy()};
  m_params.vme() = m_transportParams.vme();
  m_params.viewParamsList = m_transportParams.viewParamsList;
  m_params.frameRate = m_transportParams.frameRate;
  m_params.aaps.aaps_log2_max_afoc_present_flag(true)
      .aaps_log2_max_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4())
      .aaps_extension_present_flag(true)
      .aaps_miv_extension_present_flag(true)
      .aaps_miv_extension()
      .aame_vui_params_present_flag(true)
      .vui_parameters(vuiParameters());
  m_params.viewingSpace = m_transportParams.viewingSpace;

  setGiGeometry3dCoordinatesBitdepthMinus1();

  // Register pruning relation
  m_pruner->registerPruningRelation(m_params);

  // Turn on occupancy coding per view
  enableOccupancyPerView();

  // Set-up ASPS and AFPS
  prepareIvau();
}

// Calculate atlas frame sizes [MPEG/M52994 v2]
auto Encoder::calculateNominalAtlasFrameSizes(const MivBitstream::EncoderParams &params) const
    -> Common::SizeVector {
  if (m_maxBlockRate == 0) {
    // No constraints: one atlas per transport view
    auto result = Common::SizeVector(params.viewParamsList.size());
    std::transform(std::cbegin(params.viewParamsList), std::cend(params.viewParamsList),
                   std::begin(result),
                   [](const MivBitstream::ViewParams &x) { return x.ci.projectionPlaneSize(); });
    return result;
  }

  if (!m_overrideAtlasFrameSizes.empty()) {
    std::cout
        << "WARNING: When overriding nominal atlas frame sizes, constraints are not checked.\n";
    return m_overrideAtlasFrameSizes;
  }

  // Translate block rate into a maximum number of blocks
  const auto maxBlocks = static_cast<int>(m_maxBlockRate / params.frameRate);

  // Calculate the number of atlases
  auto numAtlases = (maxBlocks + m_maxBlocksPerAtlas - 1) / m_maxBlocksPerAtlas;
  if (numAtlases > m_maxAtlases) {
    std::cout << "The maxAtlases constraint is a limiting factor.\n";
    numAtlases = m_maxAtlases;
  }

  // Calculate the number of blocks per atlas
  auto maxBlocksPerAtlas = maxBlocks / numAtlases;
  if (maxBlocksPerAtlas > m_maxBlocksPerAtlas) {
    std::cout << "The maxLumaPictureSize constraint is a limiting factor.\n";
    maxBlocksPerAtlas = m_maxBlocksPerAtlas;
  }

  // Take the smallest reasonable width
  const auto viewGridSize = calculateViewGridSize(params);
  const auto atlasGridWidth = viewGridSize.x();
  const auto atlasGridHeight = maxBlocksPerAtlas / atlasGridWidth;

  // Warn if the aspect ratio is outside of HEVC limits (unlikely)
  if (atlasGridWidth * 8 < atlasGridHeight || atlasGridHeight * 8 < atlasGridWidth) {
    std::cout << "WARNING: Atlas aspect ratio is outside of HEVC general tier and level limits\n";
  }

  return Common::SizeVector(numAtlases,
                            {atlasGridWidth * m_blockSize, atlasGridHeight * m_blockSize});
}

auto Encoder::calculateViewGridSize(const MivBitstream::EncoderParams &params) const
    -> Common::Vec2i {
  int x{};
  int y{};

  for (const auto &viewParams : params.viewParamsList) {
    x = std::max(x, (viewParams.ci.ci_projection_plane_width_minus1() + m_blockSize) / m_blockSize);
    y = std::max(y,
                 (viewParams.ci.ci_projection_plane_height_minus1() + m_blockSize) / m_blockSize);
  }

  return {x, y};
}

auto Encoder::vuiParameters() const -> MivBitstream::VuiParameters {
  auto numUnitsInTick = 1;
  auto timeScale = static_cast<int>(numUnitsInTick * m_params.frameRate);
  LIMITATION(timeScale == numUnitsInTick * m_params.frameRate);

  auto vui = MivBitstream::VuiParameters{};
  vui.vui_timing_info_present_flag(true)
      .vui_num_units_in_tick(numUnitsInTick)
      .vui_time_scale(timeScale)
      .vui_poc_proportional_to_timing_flag(false)
      .vui_hrd_parameters_present_flag(false);
  vui.vui_unit_in_metres_flag(true);
  vui.vui_coordinate_system_parameters_present_flag(true).coordinate_system_parameters() = {};
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
    m_params.vps.geometry_information(j).gi_geometry_3d_coordinates_bit_depth_minus1(numBitsMinus1);
  }
}

auto Encoder::haveTexture() const -> bool {
  assert(m_transportParams.vps.vps_atlas_count_minus1() == 0);
  const auto j0 = m_transportParams.vps.vps_atlas_id(0);
  const auto &ai = m_transportParams.vps.attribute_information(j0);
  return ai.ai_attribute_count() >= 1 &&
         ai.ai_attribute_type_id(0) == MivBitstream::AiAttributeTypeId::ATTR_TEXTURE;
}

auto Encoder::haveOccupancy() const -> bool { return m_explicitOccupancy; }

void Encoder::enableOccupancyPerView() {
  for (size_t viewId = 0; viewId < m_params.viewParamsList.size(); ++viewId) {
    if (!m_params.viewParamsList[viewId].isBasicView ||
        m_params.vme().vme_max_entities_minus1() > 0) {
      m_params.viewParamsList[viewId].hasOccupancy = true;
    }
  }
}

void Encoder::prepareIvau() {
  m_params.atlas.resize(m_params.vps.vps_atlas_count_minus1() + size_t(1));

  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    auto &atlas = m_params.atlas[k];
    const auto j = m_params.vps.vps_atlas_id(k);
    const auto &gi = m_params.vps.geometry_information(j);

    // Set ASPS parameters
    atlas.asps.asps_frame_width(m_params.vps.vps_frame_width(j))
        .asps_frame_height(m_params.vps.vps_frame_height(j))
        .asps_geometry_3d_bit_depth_minus1(gi.gi_geometry_3d_coordinates_bit_depth_minus1())
        .asps_geometry_2d_bit_depth_minus1(gi.gi_geometry_2d_bit_depth_minus1())
        .asps_log2_max_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4())
        .asps_use_eight_orientations_flag(true)
        .asps_extended_projection_enabled_flag(true)
        .asps_normal_axis_limits_quantization_enabled_flag(true)
        .asps_max_number_projections_minus1(uint16_t(m_params.viewParamsList.size() - 1))
        .asps_log2_patch_packing_block_size(Common::ceilLog2(m_blockSize));

    // Signalling pdu_entity_id requires ASME to be present
    if (m_params.vps.vps_miv_extension_present_flag() &&
        m_params.vme().vme_max_entities_minus1() > 0) {
      // There is nothing entity-related in ASME so a reference is obtained but discarded
      static_cast<void>(atlas.asme());
    }

    // Set ATH parameters
    atlas.ath.ath_ref_atlas_frame_list_asps_flag(true);
    atlas.ath.ath_pos_min_d_quantizer( // make pdu_3d_offset_d u(0)
        atlas.asps.asps_geometry_3d_bit_depth_minus1() + 1);
  }
}

auto Encoder::log2FocLsbMinus4() -> std::uint8_t {
  // Avoid confusion but test MSB/LSB logic in decoder
  return std::max(4U, Common::ceilLog2(m_intraPeriod) + 1U) - 4U;
}
} // namespace TMIV::Encoder
