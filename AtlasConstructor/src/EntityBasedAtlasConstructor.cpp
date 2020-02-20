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
#include <cassert>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

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

  // Keep the config info to load the EntityMaps
  m_rootNode = rootNode;

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
  if (ivSequenceParams.msp().msp_max_entities_minus1() == 0) {
    m_nbAtlas =
        max(static_cast<size_t>(count(isBasicView.begin(), isBasicView.end(), true)), m_nbAtlas);
  }

  m_inIvSequenceParams = move(ivSequenceParams);

  // Create IVS with VPS with right number of atlases but copy other parts from input IVS
  m_outIvSequenceParams = IvSequenceParams{SizeVector(m_nbAtlas, m_atlasSize)};
  m_outIvSequenceParams.msp() = m_inIvSequenceParams.msp();
  m_outIvSequenceParams.viewParamsList = m_inIvSequenceParams.viewParamsList;
  m_outIvSequenceParams.viewingSpace = m_inIvSequenceParams.viewingSpace;

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
    MivBitstream::IvAccessUnitParams ivAccessUnitParams) {
  m_ivAccessUnitParams = ivAccessUnitParams;
  m_viewBuffer.clear();
  m_aggregatedEntityMask.clear();
  m_entityMasksBuffer.clear();
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

void EntityBasedAtlasConstructor::mergeViews(MVD16Frame &mergedViews,
                                             MVD16Frame transportEntityViews) {
  for (size_t viewId = 0; viewId < mergedViews.size(); viewId++) {
    for (int planeId = 0; planeId < 3; planeId++) {
      vector<int> Indices(transportEntityViews[viewId].first.getPlane(planeId).size());
      std::iota(Indices.begin(), Indices.end(), 0);
      std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
        if (transportEntityViews[viewId].first.getPlane(planeId)[i] != neutralChroma) {
          mergedViews[viewId].first.getPlane(planeId)[i] =
              transportEntityViews[viewId].first.getPlane(planeId)[i];
        }
      });
    }

    vector<int> Indices(transportEntityViews[viewId].second.getPlane(0).size());
    std::iota(Indices.begin(), Indices.end(), 0);
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if (transportEntityViews[viewId].second.getPlane(0)[i] != uint16_t(0)) {
        mergedViews[viewId].second.getPlane(0)[i] =
            transportEntityViews[viewId].second.getPlane(0)[i];
      }
    });
  }
}

void EntityBasedAtlasConstructor::mergeMasks(MaskList &mergedMasks, MaskList masks) {
  for (size_t viewId = 0; viewId < mergedMasks.size(); viewId++) {
    vector<int> Indices(mergedMasks[viewId].getPlane(0).size());
    std::iota(Indices.begin(), Indices.end(), 0);
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if (masks[viewId].getPlane(0)[i] != uint8_t(0)) {
        mergedMasks[viewId].getPlane(0)[i] = masks[viewId].getPlane(0)[i];
      }
    });
  }
}

void EntityBasedAtlasConstructor::updateMasks(const MVD16Frame &views, MaskList &masks) {
  for (size_t viewId = 0; viewId < views.size(); viewId++) {
    vector<int> Indices(masks[viewId].getPlane(0).size());
    std::iota(Indices.begin(), Indices.end(), 0);
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if ((views[viewId].first.getPlane(0)[i] == neutralChroma) &&
          (views[viewId].second.getPlane(0)[i] == uint16_t(0))) {
        masks[viewId].getPlane(0)[i] = uint8_t(0);
      }
    });
  }
}

void EntityBasedAtlasConstructor::updateEntityMasks(EntityMapList &entityMasks,
                                                    const MaskList &masks, uint16_t entityId) {
  if (entityId == 0) {
    entityId = m_inIvSequenceParams.msp().msp_max_entities_minus1() +
               1; // to avoid getting lost with the initalized 0s
  }
  for (size_t viewId = 0; viewId < entityMasks.size(); viewId++) {
    vector<int> Indices(entityMasks[viewId].getPlane(0).size());
    std::iota(Indices.begin(), Indices.end(), 0);
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if (masks[viewId].getPlane(0)[i] != uint8_t(0)) {
        entityMasks[viewId].getPlane(0)[i] = entityId;
      }
    });
  }
}

void EntityBasedAtlasConstructor::swap0(EntityMapList &entityMasks) {
  for (auto &entityMask : entityMasks) {
    vector<int> Indices(entityMask.getPlane(0).size());
    std::iota(Indices.begin(), Indices.end(), 0);
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if (entityMask.getPlane(0)[i] == uint8_t(0)) {
        entityMask.getPlane(0)[i] = unusedPatchId;
      }
    });
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if (entityMask.getPlane(0)[i] == m_inIvSequenceParams.msp().msp_max_entities_minus1() + 1) {
        entityMask.getPlane(0)[i] = uint16_t(0);
      }
    });
  }
}

void EntityBasedAtlasConstructor::aggregateEntityMasks(EntityMapList &entityMasks) {
  if (m_aggregatedEntityMask.empty()) {
    m_aggregatedEntityMask = entityMasks;
  } else {
    for (size_t i = 0; i < entityMasks.size(); i++) {
      transform(m_aggregatedEntityMask[i].getPlane(0).begin(),
                m_aggregatedEntityMask[i].getPlane(0).end(), entityMasks[i].getPlane(0).begin(),
                m_aggregatedEntityMask[i].getPlane(0).begin(),
                [](auto v1, auto v2) { return max(v1, v2); });
    }
  }
}

auto EntityBasedAtlasConstructor::entitySeparator(MVD16Frame transportViews,
                                                  EntityMapList entityMaps, uint16_t entityId)
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
  MVD16Frame mergedViews;
  MaskList masks;
  MaskList mergedMasks;
  EntityMapList entityMasks;
  for (auto &transportView : transportViews) {
    TextureDepth16Frame entityMergedView = {
        TextureFrame(transportView.first.getWidth(), transportView.first.getHeight()),
        Depth16Frame(transportView.second.getWidth(), transportView.second.getHeight())};
    Mask entityMergedMask(transportView.first.getWidth(), transportView.first.getHeight());
    EntityMap entityMask(transportView.first.getWidth(), transportView.first.getHeight());
    for (auto &p : entityMergedView.first.getPlanes()) {
      fill(p.begin(), p.end(), neutralChroma);
    }
    fill(entityMergedView.second.getPlane(0).begin(), entityMergedView.second.getPlane(0).end(),
         uint16_t(0));
    fill(entityMergedMask.getPlane(0).begin(), entityMergedMask.getPlane(0).end(), uint8_t(0));
    fill(entityMask.getPlane(0).begin(), entityMask.getPlane(0).end(), uint16_t(0));

    mergedViews.push_back(move(entityMergedView));
    mergedMasks.push_back(move(entityMergedMask));
    entityMasks.push_back(move(entityMask));
  }

  // Entity Maps Loader
  SizeVector m_viewSizes = m_inIvSequenceParams.viewParamsList.viewSizes();

  EntityMapList entityMaps;
  for (const auto &transportView : transportViews) {
    entityMaps.push_back(transportView.entities);
  }

  for (auto entityId = m_EntityEncRange[0]; entityId < m_EntityEncRange[1]; entityId++) {
    cout << "Processing entity " << entityId << '\n';

    // Entity Separator
    transportEntityViews = entitySeparator(transportViews, entityMaps, entityId);

    // Pruning
    masks = m_pruner->prune(m_inIvSequenceParams, transportEntityViews, m_isBasicView);

    // updating the pruned basic masks for entities and filter other masks.
    updateMasks(transportEntityViews, masks);

    // Entity Masking and Merging (Tracking entityIds after pruning)
    updateEntityMasks(entityMasks, masks, entityId);
    mergeViews(mergedViews, transportEntityViews);
    mergeMasks(mergedMasks, masks);
  }
  // Aggregation
  m_viewBuffer.push_back(move(mergedViews));
  m_aggregator->pushMask(mergedMasks);
  aggregateEntityMasks(entityMasks);
  m_entityMasksBuffer.push_back(move(entityMasks));

  m_fIndex++;
}

auto EntityBasedAtlasConstructor::completeAccessUnit() -> const IvAccessUnitParams & {
  m_maxEntities = m_inIvSequenceParams.msp().msp_max_entities_minus1() + 1;

  // Aggregated mask
  m_aggregator->completeAccessUnit();
  const MaskList &aggregatedMask = m_aggregator->getAggregatedMask();
  swap0(m_aggregatedEntityMask);
  for (auto &i : m_entityMasksBuffer) {
    swap0(i);
  }

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
    atlas.asps.asps_frame_width(m_atlasSize.x()).asps_frame_height(m_atlasSize.y());
  }
  m_packer->updateAggregatedEntityMasks(m_aggregatedEntityMask);
  m_ivAccessUnitParams.patchParamsList =
      m_packer->pack(m_ivAccessUnitParams.atlasSizes(), aggregatedMask, m_isBasicView);

  // Atlas construction
  m_frameInGOPIndex = 0;
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

    for (const auto &patch : m_ivAccessUnitParams.patchParamsList) {
      writePatchInAtlas(patch, views, atlasList);
    }

    m_atlasBuffer.push_back(move(atlasList));
    m_frameInGOPIndex++;
  }

  return m_ivAccessUnitParams;
}

auto EntityBasedAtlasConstructor::popAtlas() -> MVD16Frame {
  MVD16Frame atlas = move(m_atlasBuffer.front());
  m_atlasBuffer.pop_front();
  return atlas;
}

auto EntityBasedAtlasConstructor::setView(TextureDepth16Frame view, const EntityMap &entityMask,
                                          int entityId) -> TextureDepth16Frame {
  TextureDepth16Frame entityView = {TextureFrame(view.first.getWidth(), view.first.getHeight()),
                                    Depth16Frame(view.second.getWidth(), view.second.getHeight())};
  for (auto &p : entityView.first.getPlanes()) {
    fill(p.begin(), p.end(), neutralChroma);
  }
  fill(entityView.second.getPlane(0).begin(), entityView.second.getPlane(0).end(), uint16_t(0));

  EntityMapList entityMasks;
  entityMasks.push_back(entityMask);
  auto entityMasksYUV = yuvSampler(entityMasks);
  for (int planeId = 0; planeId < 3; planeId++) {
    vector<int> Indices(view.first.getPlane(planeId).size());
    std::iota(Indices.begin(), Indices.end(), 0);
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if (entityMasksYUV[0].getPlane(planeId)[i] == entityId) {
        entityView.first.getPlane(planeId)[i] = view.first.getPlane(planeId)[i];
        if (planeId == 0) {
          entityView.second.getPlane(0)[i] = view.second.getPlane(0)[i];
        }
      }
    });
  }

  return entityView;
}

void EntityBasedAtlasConstructor::writePatchInAtlas(const PatchParams &patch,
                                                    const MVD16Frame &views, MVD16Frame &atlas) {
  auto &currentAtlas = atlas[patch.vuhAtlasId];

  TextureDepth16Frame currentView;
  if (m_maxEntities > 1) {
    currentView =
        setView(views[patch.pduViewId()], m_entityMasksBuffer[m_frameInGOPIndex][patch.pduViewId()],
                *patch.pduEntityId());
  } else {
    currentView = views[patch.pduViewId()];
  }

  auto &textureAtlasMap = currentAtlas.first;
  auto &depthAtlasMap = currentAtlas.second;

  const auto &textureViewMap = currentView.first;
  const auto &depthViewMap = currentView.second;
  int w = patch.pduViewSize().x();
  int h = patch.pduViewSize().y();
  int xM = patch.pduViewPos().x();
  int yM = patch.pduViewPos().y();

  const auto &inViewParams = m_inIvSequenceParams.viewParamsList[patch.pduViewId()];
  const auto &outViewParams = m_outIvSequenceParams.viewParamsList[patch.pduViewId()];

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
