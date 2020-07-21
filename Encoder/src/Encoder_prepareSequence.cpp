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

#include <cassert>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Encoder {
void Encoder::prepareSequence(EncoderParams sourceParams) {
    if (sourceParams.vme().vme_depth_low_quality_flag())
      m_alignment = m_blockSize = 32;
    else
      m_alignment = m_blockSize = 16;
    const auto lumaSamplesPerAtlasSample = m_geometryScaleEnabledFlag ? 1.25 : 2.;
    m_maxBlockRate = m_maxLumaSampleRate / ((sourceParams.vme().vme_num_groups_minus1() + 1) *
                                            lumaSamplesPerAtlasSample * sqr(m_blockSize));
    m_maxBlocksPerAtlas = m_maxLumaPictureSize / sqr(m_blockSize);

  // Transform source to transport view sequence parameters
  m_transportParams = m_viewOptimizer->optimizeParams(move(sourceParams));

  // Calculate nominal atlas frame sizes
  const auto atlasFrameSizes = calculateNominalAtlasFrameSizes(m_transportParams);
  cout << "Nominal atlas frame sizes: { ";
  for (auto &size : atlasFrameSizes) {
    cout << ' ' << size;
  }
  cout << " }\n";

  // Create IVS with VPS with right number of atlases but copy other parts from input IVS
  m_params = EncoderParams{atlasFrameSizes, haveTexture(), haveOccupancy()};
  m_params.aaps.aaps_log2_max_afoc_present_flag(true)
      .aaps_log2_max_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4());
  m_params.vme() = m_transportParams.vme();
  m_params.viewParamsList = m_transportParams.viewParamsList;
  m_params.viewingSpace = m_transportParams.viewingSpace;
  m_params.frameRate = m_transportParams.frameRate;

  setGiGeometry3dCoordinatesBitdepthMinus1();

  // Register pruning relation
  m_pruner->registerPruningRelation(m_params);

  // Turn on occupancy coding per view
  enableOccupancyPerView();

  // Set-up ASPS and AFPS
  prepareIvau();
}

// Calculate atlas frame sizes [MPEG/M52994 v2]
auto Encoder::calculateNominalAtlasFrameSizes(const EncoderParams &params) const -> SizeVector {
  if (m_maxBlockRate == 0) {
    // No constraints: one atlas per transport view
    auto result = SizeVector(params.viewParamsList.size());
    transform(cbegin(params.viewParamsList), cend(params.viewParamsList), begin(result),
              [](const ViewParams &x) { return x.ci.projectionPlaneSize(); });
    return result;
  }

  if (!m_overrideAtlasFrameSizes.empty()) {
    cout << "WARNING: When overriding nominal atlas frame sizes, constraints are not checked.\n";
    return m_overrideAtlasFrameSizes;
  }

  // Translate block rate into a maximum number of blocks
  const auto maxBlocks = int(m_maxBlockRate / params.frameRate);

  // Calculate the number of atlases
  auto numAtlases = (maxBlocks + m_maxBlocksPerAtlas - 1) / m_maxBlocksPerAtlas;
  if (numAtlases > m_maxAtlases) {
    cout << "The maxAtlases constraint is a limiting factor.\n";
    numAtlases = m_maxAtlases;
  }

  // Calculate the number of blocks per atlas
  auto maxBlocksPerAtlas = maxBlocks / numAtlases;
  if (maxBlocksPerAtlas > m_maxBlocksPerAtlas) {
    cout << "The maxLumaPictureSize constraint is a limiting factor.\n";
    maxBlocksPerAtlas = m_maxBlocksPerAtlas;
  }

  // Take the smallest reasonable width
  const auto viewGridSize = calculateViewGridSize(params);
  const auto atlasGridWidth = viewGridSize.x();
  const auto atlasGridHeight = maxBlocksPerAtlas / atlasGridWidth;

  // Warn if the aspect ratio is outside of HEVC limits (unlikely)
  if (atlasGridWidth * 8 < atlasGridHeight || atlasGridHeight * 8 < atlasGridWidth) {
    cout << "WARNING: Atlas aspect ratio is outside of HEVC general tier and level limits\n";
  }

  return SizeVector(numAtlases, {atlasGridWidth * m_blockSize, atlasGridHeight * m_blockSize});
}

auto Encoder::calculateViewGridSize(const EncoderParams &params) const -> Vec2i {
  int x{};
  int y{};

  for (const auto &viewParams : params.viewParamsList) {
    x = max(x, (viewParams.ci.ci_projection_plane_width_minus1() + m_blockSize) / m_blockSize);
    y = max(y, (viewParams.ci.ci_projection_plane_height_minus1() + m_blockSize) / m_blockSize);
  }

  return {x, y};
}

void Encoder::setGiGeometry3dCoordinatesBitdepthMinus1() {
  uint8_t numBitsMinus1 = 9; // Main 10
  for (auto &vp : m_params.viewParamsList) {
    const auto size = max(vp.ci.ci_projection_plane_width_minus1() + 1,
                          vp.ci.ci_projection_plane_height_minus1() + 1);
    numBitsMinus1 = max(numBitsMinus1, static_cast<uint8_t>(ceilLog2(size) - 1));
  }
  for (uint8_t atlasId = 0; atlasId <= m_params.vps.vps_atlas_count_minus1(); ++atlasId) {
    m_params.vps.geometry_information(atlasId).gi_geometry_3d_coordinates_bitdepth_minus1(
        numBitsMinus1);
  }
}

auto Encoder::haveTexture() const -> bool {
  assert(m_transportParams.vps.vps_atlas_count_minus1() == 0);
  const auto &ai = m_transportParams.vps.attribute_information(0);
  return ai.ai_attribute_count() >= 1 &&
         ai.ai_attribute_type_id(0) == AiAttributeTypeId::ATTR_TEXTURE;
}

auto Encoder::haveOccupancy() const -> bool { return m_ExplicitOccupancyCoding; }

void Encoder::enableOccupancyPerView() {
  for (size_t viewId = 0; viewId < m_params.viewParamsList.size(); ++viewId) {
    if (!m_params.viewParamsList[viewId].isBasicView ||
        m_params.vme().vme_max_entities_minus1() > 0) {
      m_params.viewParamsList[viewId].hasOccupancy = true;
    }
    if (m_ExplicitOccupancyCoding) {
      m_params.viewParamsList[viewId].dq.dq_depth_occ_map_threshold_default(0);
    }
  }
}

void Encoder::prepareIvau() {
  m_params.atlas.resize(m_params.vps.vps_atlas_count_minus1() + size_t(1));

  for (uint8_t i = 0; i <= m_params.vps.vps_atlas_count_minus1(); ++i) {
    auto &atlas = m_params.atlas[i];
    const auto &gi = m_params.vps.geometry_information(i);

    // Set ASPS parameters
    atlas.asps.asps_frame_width(m_params.vps.vps_frame_width(i))
        .asps_frame_height(m_params.vps.vps_frame_height(i))
        .asps_geometry_3d_bitdepth_minus1(gi.gi_geometry_3d_coordinates_bitdepth_minus1())
        .asps_geometry_2d_bitdepth_minus1(gi.gi_geometry_nominal_2d_bitdepth_minus1())
        .asps_log2_max_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4())
        .asps_use_eight_orientations_flag(true)
        .asps_extended_projection_enabled_flag(true)
        .asps_normal_axis_limits_quantization_enabled_flag(true)
        .asps_max_number_projections_minus1(uint16_t(m_params.viewParamsList.size() - 1))
        .asps_log2_patch_packing_block_size(ceilLog2(m_blockSize));

    // Signalling pdu_entity_id requires ASME to be present
    if (m_params.vps.vps_miv_extension_flag() && m_params.vme().vme_max_entities_minus1() > 0) {
      // There is nothing entity-related in ASME so a reference is obtained but discarded
      static_cast<void>(atlas.asme());
    }

    // Set ATH parameters
    atlas.ath.ath_ref_atlas_frame_list_sps_flag(true);
    atlas.ath.ath_pos_min_z_quantizer(gi.gi_geometry_3d_coordinates_bitdepth_minus1() + 2);
    atlas.ath.ath_patch_size_x_info_quantizer(atlas.asps.asps_log2_patch_packing_block_size());
    atlas.ath.ath_patch_size_y_info_quantizer(atlas.asps.asps_log2_patch_packing_block_size());
  }
}

auto Encoder::log2FocLsbMinus4() -> std::uint8_t {
  // Avoid confusion but test MSB/LSB logic in decoder
  return max(4U, ceilLog2(m_intraPeriod) + 1U) - 4U;
}
} // namespace TMIV::Encoder
