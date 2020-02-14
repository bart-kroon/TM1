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

#include <TMIV/AtlasConstructor/EntityBasedAtlasConstructor.h>

#include "Cluster.h"
#include <TMIV/Common/Factory.h>
#include <TMIV/Metadata/DepthOccupancyTransform.h>

#include <cassert>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::AtlasConstructor {
constexpr auto neutralChroma = TextureFrame::neutralColor();

EntityBasedAtlasConstructor::EntityBasedAtlasConstructor(const Json &rootNode,
                                                         const Json &componentNode) {
  // Components
  m_pruner = Factory<IPruner>::getInstance().create("Pruner", rootNode, componentNode);
  m_aggregator = Factory<IAggregator>::getInstance().create("Aggregator", rootNode, componentNode);
  m_packer = Factory<IPacker>::getInstance().create("Packer", rootNode, componentNode);

  // Single atlas size
  m_atlasSize = componentNode.require("AtlasResolution").asIntVector<2>();

  // Read the entity encoding range
  m_EntityEncRange = componentNode.require("EntityEncodeRange").asIntVector<2>();

  // The number of atlases is determined by the specified maximum number of luma
  // samples per frame (texture and depth combined)
  int maxLumaSamplesPerFrame = componentNode.require("MaxLumaSamplesPerFrame").asInt();
  const auto lumaSamplesPerAtlas = 2 * m_atlasSize.x() * m_atlasSize.y();
  m_nbAtlas = size_t(maxLumaSamplesPerFrame / lumaSamplesPerAtlas);
}

auto EntityBasedAtlasConstructor::prepareSequence(IvSequenceParams ivSequenceParams,
                                                  vector<bool> isBasicView)
    -> const IvSequenceParams & {
  // Construct at least the basic views
  if (ivSequenceParams.maxEntities == 1) {
    m_nbAtlas =
        max(static_cast<size_t>(count(isBasicView.begin(), isBasicView.end(), true)), m_nbAtlas);
  }

  // Copy sequence parameters + Basic view ids
  m_inIvSequenceParams = move(ivSequenceParams);
  m_outIvSequenceParams = m_inIvSequenceParams;
  m_isBasicView = move(isBasicView);

  // Register pruning relation
  m_pruner->registerPruningRelation(m_outIvSequenceParams, m_isBasicView);
  
  // Turn on occupancy coding for all views
  for (auto &x : m_outIvSequenceParams.viewParamsList) {
    x.hasOccupancy = true;
  }

  return m_outIvSequenceParams;
}

void EntityBasedAtlasConstructor::prepareAccessUnit(
    Metadata::IvAccessUnitParams ivAccessUnitParams) {
  assert(ivAccessUnitParams.atlasParamsList);
  m_ivAccessUnitParams = ivAccessUnitParams;
  m_viewBuffer.clear();
  m_aggregatedEntityMask.clear();
  m_aggregator->prepareAccessUnit();
}

auto EntityBasedAtlasConstructor::yuvSampler(const EntityMapList &in) -> vector<Frame<YUV420P16>> {
  vector<Frame<YUV420P16>> outYuvAll;
  for (const auto &viewId : in) {
    Frame<YUV420P16> outYuv(int(viewId.getWidth()), int(viewId.getHeight()));
    const auto width = viewId.getWidth();
    const auto height = viewId.getHeight();
    int step = 1;
    for (int k = 0; k < 3; ++k) {
      if (k != 0) {
        step = 2;
      }
      int rowIndex = 0;
      for (int i = 0; i != height; i = i + step) {
        int colIndex = 0;
        for (int j = 0; j != width; j = j + step) {
          outYuv.getPlane(k)(rowIndex, colIndex) = viewId.getPlane(0)(i, j);
          colIndex++;
        }
        rowIndex++;
      }
    }
    outYuvAll.push_back(outYuv);
  }
  return outYuvAll;
}

void EntityBasedAtlasConstructor::mergeMasks(MaskList &mergedMasks, MaskList masks) {
  for (size_t viewId = 0; viewId < mergedMasks.size(); viewId++) {
    for (auto i = 0; i < mergedMasks[viewId].getPlane(0).size(); i++) {
      if (masks[viewId].getPlane(0)[i] != uint8_t(0)) {
        mergedMasks[viewId].getPlane(0)[i] = masks[viewId].getPlane(0)[i];
      }
    }
  }
}

void EntityBasedAtlasConstructor::updateMasks(const MVD16Frame &views, MaskList &masks) {
  for (size_t viewId = 0; viewId < views.size(); viewId++) {
    for (auto i = 0; i < masks[viewId].getPlane(0).size(); i++) {
      if ((views[viewId].first.getPlane(0)[i] == neutralChroma) &&
          (views[viewId].second.getPlane(0)[i] == uint16_t(0))) {
        masks[viewId].getPlane(0)[i] = uint8_t(0);
      }
    }
  }
}

void EntityBasedAtlasConstructor::aggregateEntityMasks(MaskList &Masks, uint16_t entityId) {
  if (m_aggregatedEntityMask.size() < m_EntityEncRange[1] - m_EntityEncRange[0]) {
    m_aggregatedEntityMask.push_back(Masks);
  } else {
    for (size_t i = 0; i < Masks.size(); i++) {
      transform(m_aggregatedEntityMask[entityId - m_EntityEncRange[0]][i].getPlane(0).begin(),
                m_aggregatedEntityMask[entityId - m_EntityEncRange[0]][i].getPlane(0).end(),
                Masks[i].getPlane(0).begin(),
                m_aggregatedEntityMask[entityId - m_EntityEncRange[0]][i].getPlane(0).begin(),
                [](auto v1, auto v2) { return max(v1, v2); });
    }
  }
}

auto EntityBasedAtlasConstructor::entitySeparator(const MVD16Frame &transportViews, uint16_t entityId)
    -> MVD16Frame {
  // Initalize entityViews
  MVD16Frame entityViews;
  for (auto &transportView : transportViews) {
    TextureDepth16Frame entityView = {
        TextureFrame(transportView.first.getWidth(), transportView.first.getHeight()),
        Depth16Frame(transportView.second.getWidth(), transportView.second.getHeight())};

    for (auto &p : entityView.first.getPlanes()) {
      fill(p.begin(), p.end(), neutralChroma);
    }

    fill(entityView.second.getPlane(0).begin(), entityView.second.getPlane(0).end(), uint16_t(0));

    entityViews.push_back(move(entityView));
  }
  EntityMapList entityMaps;
  for (const auto &transportView : transportViews) {
    entityMaps.push_back(transportView.entities);
  }

  auto entityMapsYUV = yuvSampler(entityMaps);

  for (size_t viewId = 0; viewId < transportViews.size(); viewId++) {
    for (int planeId = 0; planeId < transportViews[viewId].first.getNumberOfPlanes();
         planeId++) {                                                        //
      std::transform(transportViews[viewId].first.getPlane(planeId).begin(), // i's
                     transportViews[viewId].first.getPlane(planeId).end(),   //
                     entityMapsYUV[viewId].getPlane(planeId).begin(),        // j's
                     entityViews[viewId].first.getPlane(planeId).begin(),    // result
                     [=](auto i, auto j) { return (j == entityId) ? i : neutralChroma; });
    }
    std::transform(transportViews[viewId].second.getPlane(0).begin(), // i's
                   transportViews[viewId].second.getPlane(0).end(),   //
                   entityMaps[viewId].getPlane(0).begin(),            // j's
                   entityViews[viewId].second.getPlane(0).begin(),    // result
                   [=](auto i, auto j) { return (j == entityId) ? i : uint16_t(0); });
  }

  return entityViews;
}

void EntityBasedAtlasConstructor::pushFrame(MVD16Frame transportViews) {
  // Initalization
  MVD16Frame transportEntityViews;
  MaskList masks;
  MaskList mergedMasks;
  for (auto &transportView : transportViews) {
    Mask entityMergedMask(transportView.first.getWidth(), transportView.first.getHeight());

    fill(entityMergedMask.getPlane(0).begin(), entityMergedMask.getPlane(0).end(), uint8_t(0));

    mergedMasks.push_back(move(entityMergedMask));
  }

  SizeVector m_viewSizes = m_inIvSequenceParams.viewParamsList.viewSizes();

  for (auto entityId = m_EntityEncRange[0]; entityId < m_EntityEncRange[1]; entityId++) {
    cout << "Processing entity " << entityId << '\n';

    // Entity Separator
    transportEntityViews = entitySeparator(transportViews, entityId);

    // Pruning
    masks = m_pruner->prune(m_inIvSequenceParams, transportEntityViews, m_isBasicView);

    // updating the pruned basic masks for entities and filter other masks.
    updateMasks(transportEntityViews, masks);

    // Aggregate Entity Masks
    aggregateEntityMasks(masks, entityId);

    // Entity Mask Merging
    mergeMasks(mergedMasks, masks);
  }
  // Aggregation
  m_viewBuffer.push_back(move(transportViews));
  m_aggregator->pushMask(mergedMasks);
}

auto EntityBasedAtlasConstructor::completeAccessUnit() -> const IvAccessUnitParams & {
  m_maxEntities = m_inIvSequenceParams.maxEntities;

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
  assert(m_ivAccessUnitParams.atlasParamsList);
  m_ivAccessUnitParams.atlasParamsList->atlasSizes = SizeVector(m_nbAtlas, m_atlasSize);
  m_packer->updateAggregatedEntityMasks(m_aggregatedEntityMask);
  m_ivAccessUnitParams.atlasParamsList->setAtlasParamsVector(m_packer->pack(
      m_ivAccessUnitParams.atlasParamsList->atlasSizes, aggregatedMask, m_isBasicView));

  // Atlas construction
  for (const auto &views : m_viewBuffer) {
    MVD16Frame atlasList;

    for (size_t i = 0; i < m_nbAtlas; ++i) {
      TextureDepth16Frame atlas = {TextureFrame(m_atlasSize.x(), m_atlasSize.y()),
                                   Depth16Frame(m_atlasSize.x(), m_atlasSize.y())};

      for (auto &p : atlas.first.getPlanes()) {
        fill(p.begin(), p.end(), neutralChroma);
      }

      fill(atlas.second.getPlane(0).begin(), atlas.second.getPlane(0).end(), uint16_t(0));

      atlasList.push_back(move(atlas));
    }
    for (const auto &patch : *m_ivAccessUnitParams.atlasParamsList) {
      const auto &view = views[patch.viewId];
	  MVD16Frame tempViews;
      tempViews.push_back(move(view));
      const auto &entityViews = entitySeparator(tempViews, *patch.entityId); 
      writePatchInAtlas(patch, entityViews[0], atlasList);
    }
    m_atlasBuffer.push_back(move(atlasList));
  }

  return m_ivAccessUnitParams;
}

auto EntityBasedAtlasConstructor::popAtlas() -> MVD16Frame {
  MVD16Frame atlas = move(m_atlasBuffer.front());
  m_atlasBuffer.pop_front();
  return atlas;
}

void EntityBasedAtlasConstructor::writePatchInAtlas(const AtlasParameters &patch,
                                                    const TextureDepth16Frame &currentView,
                                                    MVD16Frame &atlas) {
  auto &currentAtlas = atlas[patch.atlasId];

  auto &textureAtlasMap = currentAtlas.first;
  auto &depthAtlasMap = currentAtlas.second;

  const auto &textureViewMap = currentView.first;
  const auto &depthViewMap = currentView.second;
  int w = patch.patchSizeInView.x();
  int h = patch.patchSizeInView.y();
  int xM = patch.posInView.x();
  int yM = patch.posInView.y();

  const auto &inViewParams = m_inIvSequenceParams.viewParamsList[patch.viewId];
  const auto &outViewParams = m_outIvSequenceParams.viewParamsList[patch.viewId];

  for (int dy = 0; dy < h; dy++) {
    for (int dx = 0; dx < w; dx++) {
      // get position
      Vec2i pView = {xM + dx, yM + dy};
      Vec2i pAtlas = viewToAtlas(pView, patch);
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
} // namespace TMIV::AtlasConstructor
