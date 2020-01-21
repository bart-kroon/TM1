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

#include "Cluster.h"
#include <TMIV/AtlasConstructor/EntityBasedAtlasConstructor.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/IO/IO.h>
#include <cassert>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::IO;

static int m_fIndex{0};
static ME16Frame m_aggregatedEntityMask;
static std::vector<ME16Frame> m_entityMasksBuffer;
static unsigned m_maxEntities;
static int m_frameInGOPIndex{0};

namespace TMIV::AtlasConstructor {
// TODO(BK): Interaction with merge request !91. Replace with TMIV::Common::neutralColor<FORMAT>().
constexpr auto neutralChroma = uint16_t(512);

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
  m_EntityEncRange = componentNode.require("EntityEncRange").asIntVector<2>();

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

auto EntityBasedAtlasConstructor::yuvSampler(const ME16Frame &in) -> ME16Frame_420 {
  ME16Frame_420 outYuvAll;
  for (const auto &inView : in) {
    Frame<YUV420P16> outYuv(int(inView.getWidth()), int(inView.getHeight()));
    const auto width = inView.getWidth();
    const auto height = inView.getHeight();
    int step = 1;
    for (int k = 0; k < 3; ++k) {
      if (k != 0) {
        step = 2;
      }
      int rowIndex = 0;
      for (auto i = 0; i != height; i = i + step) {
        int colIndex = 0;
        for (auto j = 0; j != width; j = j + step) {
          outYuv.getPlane(k)(rowIndex, colIndex) = inView.getPlane(0)(i, j);
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
  for (size_t vIndex = 0; vIndex < mergedViews.size(); vIndex++) {
    for (size_t pIndex = 0; pIndex < 3; pIndex++) {
      vector<int> Indices(transportEntityViews[vIndex].first.getPlane(pIndex).size());
      std::iota(Indices.begin(), Indices.end(), 0);
      std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
        if (transportEntityViews[vIndex].first.getPlane(pIndex)[i] != neutralChroma) {
          mergedViews[vIndex].first.getPlane(pIndex)[i] =
              transportEntityViews[vIndex].first.getPlane(pIndex)[i];
        }
      });
    }

    vector<int> Indices(transportEntityViews[vIndex].second.getPlane(0).size());
    std::iota(Indices.begin(), Indices.end(), 0);
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if (transportEntityViews[vIndex].second.getPlane(0)[i] != uint16_t(0)) {
        mergedViews[vIndex].second.getPlane(0)[i] =
            transportEntityViews[vIndex].second.getPlane(0)[i];
      }
    });
  }
}

void EntityBasedAtlasConstructor::mergeMasks(MaskList &mergedMasks, MaskList masks) {
  for (size_t vIndex = 0; vIndex < mergedMasks.size(); vIndex++) {
    vector<int> Indices(mergedMasks[vIndex].getPlane(0).size());
    std::iota(Indices.begin(), Indices.end(), 0);
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if (masks[vIndex].getPlane(0)[i] != uint8_t(0)) {
        mergedMasks[vIndex].getPlane(0)[i] = masks[vIndex].getPlane(0)[i];
      }
    });
  }
}

void EntityBasedAtlasConstructor::updateMasks(const MVD16Frame &views, MaskList &masks) {
  for (size_t vIndex = 0; vIndex < views.size(); vIndex++) {
    vector<int> Indices(masks[vIndex].getPlane(0).size());
    std::iota(Indices.begin(), Indices.end(), 0);
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if ((views[vIndex].first.getPlane(0)[i] == neutralChroma) &&
          (views[vIndex].second.getPlane(0)[i] == uint16_t(0))) {
        masks[vIndex].getPlane(0)[i] = uint8_t(0);
      }
    });
  }
}

void EntityBasedAtlasConstructor::updateEntityMasks(ME16Frame &entityMasks, const MaskList &masks,
                                                    uint16_t eIndex) {
  if (eIndex == 0) {
    eIndex = m_ivSequenceParams.maxEntities; // to avoid getting lost with the initalized 0s
  }
  for (size_t vIndex = 0; vIndex < entityMasks.size(); vIndex++) {
    vector<int> Indices(entityMasks[vIndex].getPlane(0).size());
    std::iota(Indices.begin(), Indices.end(), 0);
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if (masks[vIndex].getPlane(0)[i] != uint8_t(0)) {
        entityMasks[vIndex].getPlane(0)[i] = eIndex;
      }
    });
  }
}

void EntityBasedAtlasConstructor::swap0(ME16Frame &entityMasks) {
  for (auto &entityMask : entityMasks) {
    vector<int> Indices(entityMask.getPlane(0).size());
    std::iota(Indices.begin(), Indices.end(), 0);
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if (entityMask.getPlane(0)[i] == uint8_t(0)) {
        entityMask.getPlane(0)[i] = unusedPatchId;
      }
    });
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if (entityMask.getPlane(0)[i] == m_ivSequenceParams.maxEntities) {
        entityMask.getPlane(0)[i] = uint16_t(0);
      }
    });
  }
}

void EntityBasedAtlasConstructor::aggregateEntityMasks(ME16Frame &entityMasks) {
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

auto EntityBasedAtlasConstructor::entitySeparator(MVD16Frame transportViews, ME16Frame entityMaps,
                                                  uint16_t eIndex) -> MVD16Frame {
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

  ME16Frame_420 entityMapsYUV = yuvSampler(entityMaps);

  for (size_t vIndex = 0; vIndex < transportViews.size(); vIndex++) {
    for (int pIndex = 0; pIndex < transportViews[vIndex].first.getNumberOfPlanes(); pIndex++) { //
      std::transform(transportViews[vIndex].first.getPlane(pIndex).begin(), // i's
                     transportViews[vIndex].first.getPlane(pIndex).end(),   //
                     entityMapsYUV[vIndex].getPlane(pIndex).begin(),        // j's
                     entityViews[vIndex].first.getPlane(pIndex).begin(),    // result
                     [=](auto i, auto j) { return (j == eIndex) ? i : neutralChroma; });
    }
    std::transform(transportViews[vIndex].second.getPlane(0).begin(), // i's
                   transportViews[vIndex].second.getPlane(0).end(),   //
                   entityMaps[vIndex].getPlane(0).begin(),            // j's
                   entityViews[vIndex].second.getPlane(0).begin(),    // result
                   [=](auto i, auto j) { return (j == eIndex) ? i : uint16_t(0); });
  }

  return entityViews;
}

void EntityBasedAtlasConstructor::pushFrame(MVD16Frame transportViews) {
  // Initalization
  MVD16Frame transportEntityViews;
  MVD16Frame mergedViews;
  MaskList masks;
  MaskList mergedMasks;
  ME16Frame entityMasks;
  for (auto &transportView : transportViews) {
    TextureDepth16Frame entityMergedView = {
        TextureFrame(transportView.first.getWidth(), transportView.first.getHeight()),
        Depth16Frame(transportView.second.getWidth(), transportView.second.getHeight())};
    Mask entityMergedMask(transportView.first.getWidth(), transportView.first.getHeight());
    Entity16Frame entityMask(transportView.first.getWidth(), transportView.first.getHeight());
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
  SizeVector m_viewSizes = m_ivSequenceParams.viewParamsList.viewSizes();
  ME16Frame entityMaps = loadSourceEntityFrame(m_rootNode, m_viewSizes, m_fIndex);

  for (auto eIndex = m_EntityEncRange[0]; eIndex <= m_EntityEncRange[1]; eIndex++) {
    cout << "Processing entity " << eIndex << '\n';

    // Entity Separator
    transportEntityViews = entitySeparator(transportViews, entityMaps, eIndex);

    // Pruning
    masks = m_pruner->prune(m_ivSequenceParams.viewParamsList, transportEntityViews, m_isBasicView);

    // updating the pruned basic masks for entities and filter other masks.
    updateMasks(transportEntityViews, masks);

    // Entity Masking and Merging (Tracking entityIds after pruning)
    updateEntityMasks(entityMasks, masks, eIndex);
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
  m_packer->updateAggEntityMasks(m_aggregatedEntityMask);
  m_ivAccessUnitParams.atlasParamsList->setAtlasParamsVector(m_packer->pack(
      m_ivAccessUnitParams.atlasParamsList->atlasSizes, aggregatedMask, m_isBasicView));

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

    for (const auto &patch : *m_ivAccessUnitParams.atlasParamsList) {
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

auto EntityBasedAtlasConstructor::setView(TextureDepth16Frame view, const Entity16Frame &entityMask,
                                          int eIndex) -> TextureDepth16Frame {
  TextureDepth16Frame entityView = {TextureFrame(view.first.getWidth(), view.first.getHeight()),
                                    Depth16Frame(view.second.getWidth(), view.second.getHeight())};
  for (auto &p : entityView.first.getPlanes()) {
    fill(p.begin(), p.end(), neutralChroma);
  }
  fill(entityView.second.getPlane(0).begin(), entityView.second.getPlane(0).end(), uint16_t(0));

  ME16Frame entityMasks;
  entityMasks.push_back(entityMask);
  auto entityMasksYUV = yuvSampler(entityMasks);
  for (int pIndex = 0; pIndex < 3; pIndex++) {
    vector<int> Indices(view.first.getPlane(pIndex).size());
    std::iota(Indices.begin(), Indices.end(), 0);
    std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
      if (entityMasksYUV[0].getPlane(pIndex)[i] == eIndex) {
        entityView.first.getPlane(pIndex)[i] = view.first.getPlane(pIndex)[i];
        if (pIndex == 0) {
          entityView.second.getPlane(0)[i] = view.second.getPlane(0)[i];
        }
      }
    });
  }

  return entityView;
}

void EntityBasedAtlasConstructor::writePatchInAtlas(const AtlasParameters &patch,
                                                    const MVD16Frame &views, MVD16Frame &atlas) {
  auto &currentAtlas = atlas[patch.atlasId];

  TextureDepth16Frame currentView;
  if (m_maxEntities > 1) {
    currentView = setView(views[patch.viewId], m_entityMasksBuffer[m_frameInGOPIndex][patch.viewId],
                          *patch.entityId);
  } else {
    currentView = views[patch.viewId];
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
