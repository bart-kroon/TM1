/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

  // Single atlas size
  m_atlasSize = componentNode.require("AtlasResolution").asIntVector<2>();

  // The number of atlases is determined by the specified maximum number of luma
  // samples per frame (texture and depth combined)
  int maxLumaSamplesPerFrame = componentNode.require("MaxLumaSamplesPerFrame").asInt();
  const auto lumaSamplesPerAtlas = 2 * m_atlasSize.x() * m_atlasSize.y();
  m_nbAtlas = size_t(maxLumaSamplesPerFrame / lumaSamplesPerAtlas);

  if (rootNode.require("intraPeriod").asInt() > maxIntraPeriod) {
    throw runtime_error("The intraPeriod parameter cannot be greater than maxIntraPeriod.");
  }
}

auto AtlasConstructor::prepareSequence(IvSequenceParams ivSequenceParams, vector<bool> isBasicView)
    -> const IvSequenceParams & {
  // Construct at least the basic views
  m_nbAtlas =
      max(static_cast<size_t>(count(isBasicView.begin(), isBasicView.end(), true)), m_nbAtlas);

  m_inIvSequenceParams = move(ivSequenceParams);

  // Create IVS with VPS with right number of atlases but copy other parts from input IVS
  m_outIvSequenceParams = IvSequenceParams{SizeVector(m_nbAtlas, m_atlasSize)};
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
    int H = transportViews[view].first.getHeight();
    int W = transportViews[view].first.getWidth();
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
  auto sumValues = 0.;
  for (const auto &mask : aggregatedMask) {
    sumValues = accumulate(begin(mask.getPlane(0)), end(mask.getPlane(0)), sumValues);
  }
  const auto lumaSamplesPerFrame = 2. * sumValues / 255e6;
  cout << "Aggregated luma samples per frame is " << lumaSamplesPerFrame << "M\n";

  // Packing
  m_ivAccessUnitParams.atlas.resize(m_nbAtlas);
  for (auto &atlas : m_ivAccessUnitParams.atlas) {
    // Set ASPS parameters
    atlas.asps.asps_frame_width(m_atlasSize.x())
        .asps_frame_height(m_atlasSize.y())
        .asps_extended_projection_enabled_flag(true)
        .asps_max_projections_minus1(uint16_t(m_outIvSequenceParams.viewParamsList.size() - 1));

    // Record patch alignment -> asps_log2_patch_packing_block_size
    while (m_packer->getAlignment() % (2 << atlas.asps.asps_log2_patch_packing_block_size()) == 0) {
      atlas.asps.asps_log2_patch_packing_block_size(
          atlas.asps.asps_log2_patch_packing_block_size() + 1);
    }

    // Set AFPS parameters
    atlas.afps
        .afps_2d_pos_x_bit_count_minus1(ceilLog2(m_atlasSize.x()) -
                                        atlas.asps.asps_log2_patch_packing_block_size() - 1)
        .afps_2d_pos_y_bit_count_minus1(ceilLog2(m_atlasSize.y()) -
                                        atlas.asps.asps_log2_patch_packing_block_size() - 1);

    const auto maxViewSize =
        transform_reduce(begin(m_outIvSequenceParams.viewParamsList),
                         end(m_outIvSequenceParams.viewParamsList), Vec2i{},
                         [](auto a, auto b) {
                           return Vec2i{max(a.x(), b.x()), max(a.y(), b.y())};
                         },
                         [](const auto &vp) { return vp.ci.projectionPlaneSize(); });
    atlas.afps.afps_3d_pos_x_bit_count_minus1(ceilLog2(maxViewSize.x()) - 1);
    atlas.afps.afps_3d_pos_y_bit_count_minus1(ceilLog2(maxViewSize.y()) - 1);

    // Set ATGH parameters
    atlas.atgh.atgh_patch_size_x_info_quantizer(atlas.asps.asps_log2_patch_packing_block_size());
    atlas.atgh.atgh_patch_size_y_info_quantizer(atlas.asps.asps_log2_patch_packing_block_size());
  }
  m_ivAccessUnitParams.patchParamsList =
      m_packer->pack(m_ivAccessUnitParams.atlasSizes(), aggregatedMask, m_isBasicView);

  // Atlas construction
  int frame = 0;
  for (const auto &views : m_viewBuffer) {
    MVD16Frame atlasList;

    for (size_t i = 0; i < m_nbAtlas; ++i) {
      TextureDepth16Frame atlas = {TextureFrame(m_atlasSize.x(), m_atlasSize.y()),
                                   Depth16Frame(m_atlasSize.x(), m_atlasSize.y())};
      atlas.first.fillNeutral();
      atlasList.push_back(move(atlas));
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

void AtlasConstructor::writePatchInAtlas(const PatchParams &patch, const MVD16Frame &views,
                                         MVD16Frame &atlas, int frame) {

  auto &currentAtlas = atlas[patch.vuhAtlasId];
  const auto &currentView = views[patch.pduViewId()];

  auto &textureAtlasMap = currentAtlas.first;
  auto &depthAtlasMap = currentAtlas.second;

  const auto &textureViewMap = currentView.first;
  const auto &depthViewMap = currentView.second;
  int w = patch.pduViewSize().x();
  int h = patch.pduViewSize().y();
  int xM = patch.pduViewPos().x();
  int yM = patch.pduViewPos().y();

  int alignment = m_packer->getAlignment();

  const auto &inViewParams = m_inIvSequenceParams.viewParamsList[patch.pduViewId()];
  const auto &outViewParams = m_outIvSequenceParams.viewParamsList[patch.pduViewId()];

  for (int dyAligned = 0; dyAligned < h; dyAligned += alignment) {
    for (int dxAligned = 0; dxAligned < w; dxAligned += alignment) {

      bool isAggregatedMaskBlockNonEmpty = false;
      for (int dy = dyAligned; dy < dyAligned + alignment; dy++) {
        if (dy + yM >= textureViewMap.getHeight() || dy + yM < 0) {
          continue;
        }
        for (int dx = dxAligned; dx < dxAligned + alignment; dx++) {
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

      for (int dy = dyAligned; dy < dyAligned + alignment; dy++) {
        for (int dx = dxAligned; dx < dxAligned + alignment; dx++) {

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
