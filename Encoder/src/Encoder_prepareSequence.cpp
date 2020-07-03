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
auto Encoder::prepareSequence(IvSequenceParams sourceIvs) -> const IvSequenceParams & {
  // Transform source to transport view sequence parameters
  tie(m_transportIvs, m_isBasicView) = m_viewOptimizer->optimizeSequence(move(sourceIvs));

  // Calculate nominal atlas frame sizes
  const auto atlasFrameSizes = calculateNominalAtlasFrameSizes(m_transportIvs);
  cout << "Nominal atlas frame sizes: { ";
  for (auto &size : atlasFrameSizes) {
    cout << ' ' << size;
  }
  cout << " }\n";

  // Create IVS with VPS with right number of atlases but copy other parts from input IVS
  m_ivs = IvSequenceParams{atlasFrameSizes, haveTexture()};
  m_ivs.vme() = m_transportIvs.vme();
  m_ivs.viewParamsList = m_transportIvs.viewParamsList;
  m_ivs.viewingSpace = m_transportIvs.viewingSpace;
  m_ivs.frameRate = m_transportIvs.frameRate;
  setGiGeometry3dCoordinatesBitdepthMinus1();

  // Update views per atlas info
  // TODO(BK): Extract function
  m_ivs.mvpl().mvp_num_views_minus1(uint16_t(m_isBasicView.size() - 1));
  m_ivs.mvpl().mvp_atlas_count_minus1(m_ivs.vps.vps_atlas_count_minus1());
  for (uint8_t a = 0; a <= m_ivs.vps.vps_atlas_count_minus1(); ++a) {
    for (uint16_t v = 0; v <= m_ivs.mvpl().mvp_num_views_minus1(); ++v) {
      // TODO(BK): Update or set after packing to be more useful
      m_ivs.mvpl().mvp_view_enabled_in_atlas_flag(a, v, true);
      m_ivs.mvpl().mvp_view_complete_in_atlas_flag(a, v, m_isBasicView[v]);
    }
  }
  m_ivs.mvpl().mvp_explicit_view_id_flag(true);
  for (uint16_t v = 0; v <= m_ivs.mvpl().mvp_num_views_minus1(); ++v) {
    m_ivs.mvpl().mvp_view_id(v, v);
  }

  // Register pruning relation
  m_pruner->registerPruningRelation(m_ivs, m_isBasicView);

  // Turn on occupancy coding per view
  enableOccupancyPerView();

  // Further transform sequence parameters: geometry downscaling and depth/occupancy coding
  return m_geometryDownscaler.transformSequenceParams(
      m_depthOccupancy->transformSequenceParams(m_ivs));
}

// Calculate atlas frame sizes [MPEG/M52994 v2]
auto Encoder::calculateNominalAtlasFrameSizes(const IvSequenceParams &ivSequenceParams) const
    -> SizeVector {
  if (m_maxBlockRate == 0) {
    // No constraints: one atlas per transport view
    auto result = SizeVector(ivSequenceParams.viewParamsList.size());
    transform(cbegin(ivSequenceParams.viewParamsList), cend(ivSequenceParams.viewParamsList),
              begin(result), [](const ViewParams &x) { return x.ci.projectionPlaneSize(); });
    return result;
  }

  if (!m_overrideAtlasFrameSizes.empty()) {
    cout << "WARNING: When overriding nominal atlas frame sizes, constraints are not checked.\n";
    return m_overrideAtlasFrameSizes;
  }

  // Translate block rate into a maximum number of blocks
  const auto maxBlocks = int(m_maxBlockRate / ivSequenceParams.frameRate);

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
  const auto viewGridSize = calculateViewGridSize(ivSequenceParams);
  const auto atlasGridWidth = viewGridSize.x();
  const auto atlasGridHeight = maxBlocksPerAtlas / atlasGridWidth;

  // Warn if the aspect ratio is outside of HEVC limits (unlikely)
  if (atlasGridWidth * 8 < atlasGridHeight || atlasGridHeight * 8 < atlasGridWidth) {
    cout << "WARNING: Atlas aspect ratio is outside of HEVC general tier and level limits\n";
  }

  return SizeVector(numAtlases, {atlasGridWidth * m_blockSize, atlasGridHeight * m_blockSize});
}

auto Encoder::calculateViewGridSize(const IvSequenceParams &ivSequenceParams) const -> Vec2i {
  int x{};
  int y{};

  for (const auto &viewParams : ivSequenceParams.viewParamsList) {
    x = max(x, (viewParams.ci.ci_projection_plane_width_minus1() + m_blockSize) / m_blockSize);
    y = max(y, (viewParams.ci.ci_projection_plane_height_minus1() + m_blockSize) / m_blockSize);
  }

  return {x, y};
}

void Encoder::setGiGeometry3dCoordinatesBitdepthMinus1() {
  uint8_t numBitsMinus1 = 9; // Main 10
  for (auto &vp : m_ivs.viewParamsList) {
    const auto size = max(vp.ci.ci_projection_plane_width_minus1() + 1,
                          vp.ci.ci_projection_plane_height_minus1() + 1);
    numBitsMinus1 = max(numBitsMinus1, static_cast<uint8_t>(ceilLog2(size) - 1));
  }
  for (uint8_t atlasId = 0; atlasId <= m_ivs.vps.vps_atlas_count_minus1(); ++atlasId) {
    m_ivs.vps.geometry_information(atlasId).gi_geometry_3d_coordinates_bitdepth_minus1(
        numBitsMinus1);
  }
}

auto Encoder::haveTexture() const -> bool {
  assert(m_transportIvs.vps.vps_atlas_count_minus1() == 0);
  const auto &ai = m_transportIvs.vps.attribute_information(0);
  return ai.ai_attribute_count() >= 1 &&
         ai.ai_attribute_type_id(0) == AiAttributeTypeId::ATTR_TEXTURE;
}

void Encoder::enableOccupancyPerView() {
  for (size_t viewId = 0; viewId < m_ivs.viewParamsList.size(); ++viewId) {
    if (!m_isBasicView[viewId] || m_ivs.vme().vme_max_entities_minus1() > 0) {
      m_ivs.viewParamsList[viewId].hasOccupancy = true;
    }
  }
}
} // namespace TMIV::Encoder
