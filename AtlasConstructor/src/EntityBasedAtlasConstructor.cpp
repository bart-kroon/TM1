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

#include <TMIV/Common/Factory.h>

#include "Cluster.h"

#include <cassert>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::AtlasConstructor {
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
  m_EntityEncodeRange = componentNode.require("EntityEncodeRange").asIntVector<2>();

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
  m_ivSequenceParams = move(ivSequenceParams);
  m_isBasicView = move(isBasicView);

  return m_ivSequenceParams;
}

void EntityBasedAtlasConstructor::prepareAccessUnit(
    Metadata::IvAccessUnitParams ivAccessUnitParams) {
  assert(ivAccessUnitParams.atlasParamsList);
  m_ivAccessUnitParams = ivAccessUnitParams;
  m_viewBuffer.clear();
  m_aggregatedEntityMask.clear();
  m_entityMasksBuffer.clear();
  m_aggregator->prepareAccessUnit();
}

void EntityBasedAtlasConstructor::mergeViews(MVD16Frame &mergedViews,
                                             MVD16Frame transportEntityViews) {
  for (size_t viewId = 0; viewId < mergedViews.size(); viewId++) {
    for (int planeId = 0; planeId < 3; planeId++) {
      copy_if(cbegin(transportEntityViews[viewId].first.getPlane(planeId)),
              cend(transportEntityViews[viewId].first.getPlane(planeId)),
              begin(mergedViews[viewId].first.getPlane(planeId)),
              [](auto x) { return x != TextureFrame::neutralColor(); });
    }

    copy_if(cbegin(transportEntityViews[viewId].second.getPlane(0)),
            cend(transportEntityViews[viewId].second.getPlane(0)),
            begin(mergedViews[viewId].second.getPlane(0)), [](auto x) { return x != 0; });
  }
}

void EntityBasedAtlasConstructor::mergeMasks(MaskList &mergedMasks, MaskList masks) {
  for (size_t viewId = 0; viewId < mergedMasks.size(); viewId++) {
    copy_if(cbegin(masks[viewId].getPlane(0)), cend(masks[viewId].getPlane(0)),
            begin(mergedMasks[viewId].getPlane(0)), [](auto x) { return x != 0; });
  }
}

void EntityBasedAtlasConstructor::updateMasks(const MVD16Frame &views, MaskList &masks) {
  for (size_t viewId = 0; viewId < views.size(); viewId++) {
    for (size_t i = 0; i < masks[viewId].getPlane(0).size(); ++i) {
      if (views[viewId].first.getPlane(0)[i] ==
              views[viewId].first.neutralColor() /* is this necessary? */
          && views[viewId].second.getPlane(0)[i] == 0) {
        masks[viewId].getPlane(0)[i] = 0;
      }
    }
  }
}

void EntityBasedAtlasConstructor::updateEntityMasks(EntityMapList &entityMasks,
                                                    const MaskList &masks, uint16_t entityId) {
  if (entityId == 0) {
    entityId = m_ivSequenceParams.maxEntities; // to avoid getting lost with the initalized 0s
  }
  for (size_t viewId = 0; viewId < entityMasks.size(); viewId++) {
    for (size_t i = 0; i < entityMasks[viewId].getPlane(0).size(); ++i) {
      if (masks[viewId].getPlane(0)[i] != 0) {
        entityMasks[viewId].getPlane(0)[i] = entityId;
      }
    }
  }
}

void EntityBasedAtlasConstructor::swap0(EntityMapList &entityMasks) {
  for (auto &entityMask : entityMasks) {
    for (auto &x : entityMask.getPlane(0)) {
      if (x == 0) {
        x = unusedPatchId;
      } else if (x == m_ivSequenceParams.maxEntities) {
        x = 0;
      }
    }
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
                [](uint16_t v1, uint16_t v2) { return max(v1, v2); });
    }
  }
}

auto EntityBasedAtlasConstructor::maskViews(MVD16Frame transportViews, uint16_t entityId)
    -> MVD16Frame {
  for (auto &transportView : transportViews) {
    transportView = maskView(transportView, entityId);
  }
  return transportViews;
}

void EntityBasedAtlasConstructor::pushFrame(MVD16Frame transportViews) {
  // Merged views start as neutral gray
  auto mergedViews = transportViews;
  for (auto &mergedView : mergedViews) {
    mergedView.first.fillNeutral();
    mergedView.second.fillZero();
  }

  // Merged masks start all zero
  MaskList mergedMasks;
  EntityMapList entityMasks;

  for (auto &transportView : transportViews) {
    mergedMasks.emplace_back(transportView.first.getWidth(), transportView.first.getHeight());
    entityMasks.emplace_back(transportView.first.getWidth(), transportView.first.getHeight());
  }

  for (auto entityId = m_EntityEncodeRange[0]; entityId < m_EntityEncodeRange[1]; ++entityId) {
    cout << "Processing entity " << entityId << '\n';

    // Separate out other entities
    const auto entityViews = maskViews(transportViews, entityId);

    // Prune this entity
    auto masks = m_pruner->prune(m_ivSequenceParams.viewParamsList, entityViews, m_isBasicView);

    // updating the pruned basic masks for entities and filter other masks.
    updateMasks(entityViews, masks);

    // Entity Masking and Merging (Tracking entityIds after pruning)
    updateEntityMasks(entityMasks, masks, entityId);
    mergeViews(mergedViews, entityViews);
    mergeMasks(mergedMasks, masks);
  }

  // Aggregation
  m_viewBuffer.push_back(move(mergedViews));
  m_aggregator->pushMask(mergedMasks);
  aggregateEntityMasks(entityMasks);
  m_entityMasksBuffer.push_back(move(entityMasks));
}

auto EntityBasedAtlasConstructor::completeAccessUnit() -> const IvAccessUnitParams & {
  m_maxEntities = m_ivSequenceParams.maxEntities;

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
      atlas.first.fillNeutral();
      atlasList.push_back(move(atlas));
    }

    for (const auto &patch : *m_ivAccessUnitParams.atlasParamsList) {
      writePatchInAtlas(patch, views, atlasList);
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

auto EntityBasedAtlasConstructor::maskView(TextureDepth16Frame view, int entityId)
    -> TextureDepth16Frame {
  for (int i = 0; i < view.first.getHeight(); ++i) {
    for (int j = 0; j < view.first.getWidth(); ++j) {
      if (entityId != view.entities.getPlane(0)(i, j)) {
        static_assert(is_same_v<TextureFrame, Frame<YUV420P10>>); // 4:2:0 assumption
        view.first.getPlane(0)(i, j) = view.first.neutralColor();
        view.first.getPlane(1)(i / 2, j / 2) = view.first.neutralColor();
        view.first.getPlane(2)(i / 2, j / 2) = view.first.neutralColor();
      }
    }
  }
  return view;
}

void EntityBasedAtlasConstructor::writePatchInAtlas(const AtlasParameters &patch,
                                                    const MVD16Frame &views, MVD16Frame &atlas) {
  auto &currentAtlas = atlas[patch.atlasId];
  auto currentView = views[patch.viewId];
  if (m_maxEntities > 1) {
    currentView = maskView(currentView, *patch.entityId);
  }

  auto &textureAtlasMap = currentAtlas.first;
  auto &depthAtlasMap = currentAtlas.second;

  const auto &textureViewMap = currentView.first;
  const auto &depthViewMap = currentView.second;
  int w = patch.patchSizeInView.x();
  int h = patch.patchSizeInView.y();
  int xM = patch.posInView.x();
  int yM = patch.posInView.y();

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
      depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) =
          depthViewMap.getPlane(0)(pView.y(), pView.x());
    }
  }
}
} // namespace TMIV::AtlasConstructor
