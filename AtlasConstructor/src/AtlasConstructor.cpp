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
#include <TMIV/AtlasConstructor/AtlasConstructor.h>
#include <TMIV/Common/Factory.h>

#include <cassert>
#include <fstream>
#include <iterator>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::AtlasConstructor {
constexpr auto neutralChroma = uint16_t(512);

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
  m_nbAtlas = uint16_t(maxLumaSamplesPerFrame / lumaSamplesPerAtlas);
}

void AtlasConstructor::prepareIntraPeriod(ViewParamsVector basicViewParamsVector,
                                          ViewParamsVector additionalViewParamsVector) {
  m_viewParamsVector.clear();
  m_viewParamsVector.insert(m_viewParamsVector.end(),
                            make_move_iterator(begin(basicViewParamsVector)),
                            make_move_iterator(end(basicViewParamsVector)));
  m_viewParamsVector.insert(m_viewParamsVector.end(),
                            make_move_iterator(begin(additionalViewParamsVector)),
                            make_move_iterator(end(additionalViewParamsVector)));

  m_isReferenceView.clear();
  m_isReferenceView.insert(m_isReferenceView.end(), basicViewParamsVector.size(), 1);
  m_isReferenceView.insert(m_isReferenceView.end(), additionalViewParamsVector.size(), 0);

  m_viewBuffer.clear();
  m_aggregator->prepareIntraPeriod();

  m_nbAtlas = max(uint16_t(basicViewParamsVector.size()), m_nbAtlas);
}

void AtlasConstructor::pushFrame(MVD16Frame basicViews, MVD16Frame additionalViews) {
  MVD16Frame views;
  views.insert(views.end(), make_move_iterator(begin(basicViews)),
               make_move_iterator(end(basicViews)));
  views.insert(views.end(), make_move_iterator(begin(additionalViews)),
               make_move_iterator(end(additionalViews)));

  // Pruning
  MaskList masks = m_pruner->prune(m_viewParamsVector, views, m_isReferenceView);

  // Aggregation
  m_viewBuffer.push_back(move(views));
  m_aggregator->pushMask(masks);
}

void AtlasConstructor::completeIntraPeriod() {
  // Aggregated mask
  m_aggregator->completeIntraPeriod();
  const MaskList &aggregatedMask = m_aggregator->getAggregatedMask();

  // Packing
  m_patchList =
      m_packer->pack(SizeVector(m_nbAtlas, m_atlasSize), aggregatedMask, m_isReferenceView);

  // Atlas construction
  for (const auto &views : m_viewBuffer) {
    MVD16Frame atlasList;

    for (int i = 0; i < m_nbAtlas; i++) {
      TextureDepth16Frame atlas = {TextureFrame(m_atlasSize.x(), m_atlasSize.y()),
                                   Depth16Frame(m_atlasSize.x(), m_atlasSize.y())};

      for (auto &p : atlas.first.getPlanes()) {
        fill(p.begin(), p.end(), neutralChroma);
      }

      fill(atlas.second.getPlane(0).begin(), atlas.second.getPlane(0).end(), uint16_t(0));

      atlasList.push_back(move(atlas));
    }

    for (const auto &patch : m_patchList) {
      writePatchInAtlas(patch, views, atlasList);
    }

    m_atlasBuffer.push_back(move(atlasList));
  }
}

auto AtlasConstructor::getAtlasSize() const -> SizeVector {
  assert(!m_atlasBuffer.empty());
  SizeVector result;
  for (const auto &view : m_atlasBuffer.front()) {
    result.push_back({view.first.getWidth(), view.first.getHeight()});
  }
  return result;
}

auto AtlasConstructor::popAtlas() -> MVD16Frame {
  MVD16Frame atlas = move(m_atlasBuffer.front());
  m_atlasBuffer.pop_front();
  return atlas;
}

void AtlasConstructor::writePatchInAtlas(const AtlasParameters &patch, const MVD16Frame &views,
                                         MVD16Frame &atlas) {

  auto &currentAtlas = atlas[patch.atlasId];
  const auto &currentView = views[patch.viewId];

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
