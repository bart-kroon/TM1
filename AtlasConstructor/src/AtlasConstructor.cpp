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

#include <TMIV/AtlasConstructor/AtlasConstructor.h>

#include "Cluster.h"
#include <TMIV/Common/Factory.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

#include <cassert>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::AtlasConstructor {
AtlasConstructor::AtlasConstructor(const Json &rootNode, const Json &componentNode) {
  // Components
  m_pruner = Factory<IPruner>::getInstance().create("Pruner", rootNode, componentNode);
  m_aggregator = Factory<IAggregator>::getInstance().create("Aggregator", rootNode, componentNode);
  m_packer = Factory<IPacker>::getInstance().create("Packer", rootNode, componentNode);

  // Parameters
  const auto numGroups = rootNode.require("numGroups").asInt();
  m_blockSize = rootNode.require("blockSize").asInt();
  const auto maxAtlasWidth = rootNode.require("maxAtlasWidth").asInt();
  const auto maxAtlasHeight = rootNode.require("maxAtlasHeight").asInt();
  const auto maxLumaSamplesPerFrame = rootNode.require("maxLumaSamplesPerFrame").asInt();

  // Derived parameters
  m_maxBlocksPerAtlas = maxLumaSamplesPerFrame / (2 * numGroups * sqr(m_blockSize));
  m_maxAtlasGridWidth = maxAtlasWidth / m_blockSize;
  m_maxAtlasGridHeight = maxAtlasHeight / m_blockSize;

  if (rootNode.require("intraPeriod").asInt() > maxIntraPeriod) {
    throw runtime_error("The intraPeriod parameter cannot be greater than maxIntraPeriod.");
  }
}

// Calculate atlas frame sizes [MPEG/M52994]
auto AtlasConstructor::calculateNominalAtlasFrameSizes(const IvSequenceParams &ivSequenceParams,
                                                       const std::vector<bool> &isBasicView) const
    -> SizeVector {
  if (m_maxBlocksPerAtlas == 0) {
    // No luma sample count restriction: one atlas per transport view
    auto x = SizeVector(ivSequenceParams.viewParamsList.size());
    transform(cbegin(ivSequenceParams.viewParamsList), cend(ivSequenceParams.viewParamsList),
              begin(x), [this](const ViewParams &x) { return x.ci.projectionPlaneSize(); });
    return x;
  }

  // Calculate the height of the tallest view
  const int maxViewGridHeight = accumulate(
      cbegin(ivSequenceParams.viewParamsList), cend(ivSequenceParams.viewParamsList), 0,
      [this](int x, const ViewParams &vp) {
        return max(x, (vp.ci.ci_projection_plane_height_minus1() + m_blockSize) / m_blockSize);
      });

  // Make the atlas as wide as allowed but still high enough to fit the tallest basic view
  const auto atlasGridWidth = min(m_maxAtlasGridWidth, m_maxBlocksPerAtlas / maxViewGridHeight);
  const auto atlasGridHeight = m_maxBlocksPerAtlas / atlasGridWidth;
  return {Vec2i{int(atlasGridWidth * m_blockSize), int(atlasGridHeight * m_blockSize)}};
}

auto AtlasConstructor::prepareSequence(IvSequenceParams ivSequenceParams, vector<bool> isBasicView)
    -> const IvSequenceParams & {
  m_inIvSequenceParams = move(ivSequenceParams);

  // Do we also have texture or only geometry?
  assert(m_inIvSequenceParams.vps.vps_atlas_count_minus1() == 0);
  const auto &ai = m_inIvSequenceParams.vps.attribute_information(0);
  auto const haveTexture =
      ai.ai_attribute_count() >= 1 && ai.ai_attribute_type_id(0) == AiAttributeTypeId::ATTR_TEXTURE;

  // Calculate nominal atlas frame sizes
  cout << "Nominal atlas frame sizes: { ";
  const auto atlasFrameSizes = calculateNominalAtlasFrameSizes(m_inIvSequenceParams, m_isBasicView);
  for (auto &size : atlasFrameSizes) {
    cout << ' ' << size;
  }
  cout << " }\n";

  // Create IVS with VPS with right number of atlases but copy other parts from input IVS
  m_outIvSequenceParams = IvSequenceParams{atlasFrameSizes, haveTexture};
  m_outIvSequenceParams.msp() = m_inIvSequenceParams.msp();
  m_outIvSequenceParams.viewParamsList = m_inIvSequenceParams.viewParamsList;
  m_outIvSequenceParams.viewingSpace = m_inIvSequenceParams.viewingSpace;

  m_isBasicView = move(isBasicView);

  // Register pruning relation
  m_pruner->registerPruningRelation(m_outIvSequenceParams, m_isBasicView);

  // Turn on occupancy coding for partial views
  for (size_t viewId = 0; viewId < m_outIvSequenceParams.viewParamsList.size(); ++viewId) {
    if (!m_isBasicView[viewId]) {
      m_outIvSequenceParams.viewParamsList[viewId].hasOccupancy = true;
    }
  }

  return m_outIvSequenceParams;
}

void AtlasConstructor::prepareAccessUnit(MivBitstream::IvAccessUnitParams ivAccessUnitParams) {
  m_ivAccessUnitParams = ivAccessUnitParams;

  const auto numOfCam = m_inIvSequenceParams.viewParamsList.size();

  for (size_t c = 0; c < numOfCam; c++) {
    Mat<bitset<maxIntraPeriod>> nonAggMask;
    int H = m_inIvSequenceParams.viewParamsList[c].ci.projectionPlaneSize().y();
    int W = m_inIvSequenceParams.viewParamsList[c].ci.projectionPlaneSize().x();

    nonAggMask.resize(H, W);
    for (int h = 0; h < H; h++) {
      for (int w = 0; w < W; w++) {
        nonAggMask(h, w) = 0;
      }
    }
    m_nonAggregatedMask.push_back(nonAggMask);
  }

  m_viewBuffer.clear();
  m_aggregator->prepareAccessUnit();
}

void AtlasConstructor::pushFrame(MVD16Frame transportViews) {
  // Pruning
  MaskList masks = m_pruner->prune(m_inIvSequenceParams, transportViews, m_isBasicView);

  const auto frame = m_viewBuffer.size();

  for (size_t view = 0; view < masks.size(); ++view) {
    int H = transportViews[view].texture.getHeight();
    int W = transportViews[view].texture.getWidth();
    for (int h = 0; h < H; h++) {
      for (int w = 0; w < W; w++) {
        if (masks[view].getPlane(0)(h, w) != 0) {
          m_nonAggregatedMask[view](h, w)[frame] = true;
        }
      } // w
    }   // h
  }     // view

  // Aggregation
  m_viewBuffer.push_back(move(transportViews));
  m_aggregator->pushMask(masks);
}

auto AtlasConstructor::completeAccessUnit() -> const IvAccessUnitParams & {
  // Aggregated mask
  m_aggregator->completeAccessUnit();
  const MaskList &aggregatedMask = m_aggregator->getAggregatedMask();

  // Print statistics the same way the HierarchicalPruner does
  const auto lumaSamplesPerFrame = accumulate(
      aggregatedMask.begin(), aggregatedMask.end(), size_t{}, [](size_t sum, const auto &mask) {
        return sum + 2 * count_if(mask.getPlane(0).begin(), mask.getPlane(0).end(),
                                  [](auto x) { return x > 0; });
      });
  cout << "Aggregated luma samples per frame is " << (1e-6 * lumaSamplesPerFrame) << "M\n";
  m_maxLumaSamplesPerFrame = max(m_maxLumaSamplesPerFrame, lumaSamplesPerFrame);

  // Set atlas parameters
  m_ivAccessUnitParams.atlas.resize(m_outIvSequenceParams.vps.vps_atlas_count_minus1() + 1);

  for (uint8_t i = 0; i <= m_outIvSequenceParams.vps.vps_atlas_count_minus1(); ++i) {
    auto &atlas = m_ivAccessUnitParams.atlas[i];
    const auto frameWidth = m_outIvSequenceParams.vps.vps_frame_width(i);
    const auto frameHeight = m_outIvSequenceParams.vps.vps_frame_height(i);

    // Set ASPS parameters
    atlas.asps.asps_frame_width(frameWidth)
        .asps_frame_height(frameHeight)
        .asps_use_eight_orientations_flag(true)
        .asps_extended_projection_enabled_flag(true)
        .asps_max_projections_minus1(uint16_t(m_outIvSequenceParams.viewParamsList.size() - 1))
        .asps_log2_patch_packing_block_size(ceilLog2(m_blockSize));

    // Set AFPS parameters
    atlas.afps.afps_2d_pos_x_bit_count_minus1(ceilLog2(frameWidth / m_blockSize) - 1)
        .afps_2d_pos_y_bit_count_minus1(ceilLog2(frameHeight / m_blockSize) - 1);

    uint16_t maxProjectionPlaneWidthMinus1 = 0;
    uint16_t maxProjectionPlaneHeightMinus1 = 0;
    for (auto &vp : m_outIvSequenceParams.viewParamsList) {
      maxProjectionPlaneWidthMinus1 =
          max(maxProjectionPlaneWidthMinus1, vp.ci.ci_projection_plane_width_minus1());
      maxProjectionPlaneHeightMinus1 =
          max(maxProjectionPlaneHeightMinus1, vp.ci.ci_projection_plane_height_minus1());
    }
    atlas.afps.afps_3d_pos_x_bit_count_minus1(ceilLog2(maxProjectionPlaneWidthMinus1 + 1) - 1);
    atlas.afps.afps_3d_pos_y_bit_count_minus1(ceilLog2(maxProjectionPlaneHeightMinus1 + 1) - 1);

    // Set ATGH parameters
    atlas.atgh.atgh_patch_size_x_info_quantizer(atlas.asps.asps_log2_patch_packing_block_size());
    atlas.atgh.atgh_patch_size_y_info_quantizer(atlas.asps.asps_log2_patch_packing_block_size());
  }

  // Packing
  m_ivAccessUnitParams.patchParamsList =
      m_packer->pack(m_ivAccessUnitParams.atlasSizes(), aggregatedMask, m_isBasicView);

  // Atlas construction
  int frame = 0;
  for (const auto &views : m_viewBuffer) {
    MVD16Frame atlasList;

    for (const auto &atlas : m_ivAccessUnitParams.atlas) {
      auto texture = TextureFrame(atlas.asps.asps_frame_width(), atlas.asps.asps_frame_height());
      auto depth = Depth16Frame(atlas.asps.asps_frame_width(), atlas.asps.asps_frame_height());
      texture.fillNeutral();
      atlasList.emplace_back(move(texture), move(depth));
    }

    for (const auto &patch : m_ivAccessUnitParams.patchParamsList) {
      writePatchInAtlas(patch, views, atlasList, frame);
    }

    m_atlasBuffer.push_back(move(atlasList));
    frame++;
  }

  return m_ivAccessUnitParams;
}

auto AtlasConstructor::popAtlas() -> MVD16Frame {
  MVD16Frame atlas = move(m_atlasBuffer.front());
  m_atlasBuffer.pop_front();
  return atlas;
}

auto AtlasConstructor::maxLumaSamplesPerFrame() const -> size_t { return m_maxLumaSamplesPerFrame; }

void AtlasConstructor::writePatchInAtlas(const PatchParams &patch, const MVD16Frame &views,
                                         MVD16Frame &atlas, int frame) {

  auto &currentAtlas = atlas[patch.vuhAtlasId];
  const auto &currentView = views[patch.pduViewId()];

  auto &textureAtlasMap = currentAtlas.texture;
  auto &depthAtlasMap = currentAtlas.depth;

  const auto &textureViewMap = currentView.texture;
  const auto &depthViewMap = currentView.depth;
  int w = patch.pduViewSize().x();
  int h = patch.pduViewSize().y();
  int xM = patch.pduViewPos().x();
  int yM = patch.pduViewPos().y();

  const auto &inViewParams = m_inIvSequenceParams.viewParamsList[patch.pduViewId()];
  const auto &outViewParams = m_outIvSequenceParams.viewParamsList[patch.pduViewId()];

  for (int dyAligned = 0; dyAligned < h; dyAligned += m_blockSize) {
    for (int dxAligned = 0; dxAligned < w; dxAligned += m_blockSize) {

      bool isAggregatedMaskBlockNonEmpty = false;
      for (int dy = dyAligned; dy < dyAligned + m_blockSize; dy++) {
        if (dy + yM >= textureViewMap.getHeight() || dy + yM < 0) {
          continue;
        }
        for (int dx = dxAligned; dx < dxAligned + m_blockSize; dx++) {
          if (dx + xM >= textureViewMap.getWidth() || dx + xM < 0) {
            continue;
          }
          if (m_nonAggregatedMask[patch.pduViewId()](dy + yM, dx + xM)[frame]) {
            isAggregatedMaskBlockNonEmpty = true;
            break;
          }
        }
        if (isAggregatedMaskBlockNonEmpty) {
          break;
        }
      }

      for (int dy = dyAligned; dy < dyAligned + m_blockSize; dy++) {
        for (int dx = dxAligned; dx < dxAligned + m_blockSize; dx++) {

          Vec2i pView = {xM + dx, yM + dy};
          Vec2i pAtlas = patch.viewToAtlas(pView);

          if (pView.y() >= textureViewMap.getHeight() || pView.x() >= textureViewMap.getWidth() ||
              pAtlas.y() >= textureAtlasMap.getHeight() ||
              pAtlas.x() >= textureAtlasMap.getWidth() || pView.y() < 0 || pView.x() < 0 ||
              pAtlas.y() < 0 || pAtlas.x() < 0) {
            continue;
          }

          if (!isAggregatedMaskBlockNonEmpty) {
            depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
            continue;
          }

          // Y
          textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) =
              textureViewMap.getPlane(0)(pView.y(), pView.x());
          // UV
          if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
            for (int p = 1; p < 3; ++p) {
              textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2) =
                  textureViewMap.getPlane(p)(pView.y() / 2, pView.x() / 2);
            }
          }

          // Depth
          auto depth = depthViewMap.getPlane(0)(pView.y(), pView.x());
          if (depth == 0 && !inViewParams.hasOccupancy && outViewParams.hasOccupancy) {
            depth = 1; // Avoid marking valid depth as invalid
          }
          depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = depth;
        }
      }
    }
  }
}
} // namespace TMIV::AtlasConstructor
