/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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
#include <iostream>
#include <iterator>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::AtlasConstructor {

AtlasConstructor::AtlasConstructor(const Common::Json &node) {
  // Components
  m_pruner =
      Factory<IPruner>::getInstance().create("Pruner", node.require("Pruner"));
  m_aggregator = Factory<IAggregator>::getInstance().create(
      "Aggregator", node.require("Aggregator"));
  m_packer =
      Factory<IPacker>::getInstance().create("Packer", node.require("Packer"));

  // Single atlas size
  if (auto subnode = node.optional("AtlasResolution"))
    m_atlasSize = subnode.asIntVector<2>();

  // Maximum pixel rate per frame (Texture or Depth)
  int maxMegaPixelPerFrame = 7680 * 4320 / (1000000); // 8K UHD

  if (auto subnode = node.optional("MPixel"))
    maxMegaPixelPerFrame = subnode.asInt();

  m_nbAtlas = static_cast<uint16_t>(ceil((float)maxMegaPixelPerFrame * 1000000 /
                                         (m_atlasSize.x() * m_atlasSize.y())));
}

void AtlasConstructor::prepareIntraPeriod(
    CameraParameterList baseCameras, CameraParameterList additionalCameras) {
  m_cameras.clear();
  m_cameras.insert(m_cameras.end(), make_move_iterator(begin(baseCameras)),
                   make_move_iterator(end(baseCameras)));
  m_cameras.insert(m_cameras.end(),
                   make_move_iterator(begin(additionalCameras)),
                   make_move_iterator(end(additionalCameras)));

  m_isReferenceView.clear();
  m_isReferenceView.insert(m_isReferenceView.end(), baseCameras.size(), 1);
  m_isReferenceView.insert(m_isReferenceView.end(), additionalCameras.size(),
                           0);

  m_aggregator->prepareIntraPeriod();
}

void AtlasConstructor::pushFrame(MVD16Frame baseViews,
                                 MVD16Frame additionalViews) {
  MVD16Frame views;
  views.insert(views.end(), make_move_iterator(begin(baseViews)),
               make_move_iterator(end(baseViews)));
  views.insert(views.end(), make_move_iterator(begin(additionalViews)),
               make_move_iterator(end(additionalViews)));

  // Pruning
  MaskList masks = m_pruner->prune(m_cameras, views, m_isReferenceView);

  // Aggregation
  m_viewBuffer.push_back(std::move(views));
  m_aggregator->pushMask(masks);
}

void AtlasConstructor::completeIntraPeriod() {
  // Aggregated mask
  m_aggregator->completeIntraPeriod();
  const MaskList &aggregatedMask = m_aggregator->getAggregatedMask();

  // Packing
  m_patchList = m_packer->pack(std::vector<Vec2i>(m_nbAtlas, m_atlasSize),
                               aggregatedMask, m_isReferenceView);

  // Atlas construction
  for (const auto &views : m_viewBuffer) {
    MVD16Frame atlasList;

    for (int i = 0; i < m_nbAtlas; i++) {
      TextureDepth16Frame atlas = {
          TextureFrame(m_atlasSize.x(), m_atlasSize.y()),
          Depth16Frame(m_atlasSize.x(), m_atlasSize.y())};

      for (auto &p : atlas.first.getPlanes())
        std::fill(p.begin(), p.end(), uint16_t(512));

      std::fill(atlas.second.getPlane(0).begin(),
                atlas.second.getPlane(0).end(), uint16_t(0));

      atlasList.push_back(std::move(atlas));
    }

    for (const auto &patch : m_patchList)
      writePatchInAtlas(patch, views, atlasList);

    m_atlasBuffer.push_back(std::move(atlasList));
  }
}

vector<Vec2i> AtlasConstructor::getAtlasSize() const {
  assert(!m_atlasBuffer.empty());
  vector<Vec2i> result;
  for (const auto &view : m_atlasBuffer.front()) {
    result.push_back({view.first.getWidth(), view.first.getHeight()});
  }
  return result;
}

Common::MVD16Frame AtlasConstructor::popAtlas() {
  Common::MVD16Frame atlas = std::move(m_atlasBuffer.front());
  m_atlasBuffer.pop_front();
  return atlas;
}

void AtlasConstructor::writePatchInAtlas(const PatchParameters &patch,
                                         const MVD16Frame &views,
                                         MVD16Frame &atlas) {
  auto &currentAtlas = atlas[patch.atlasId];
  auto &currentView = views[patch.virtualCameraId];

  auto &textureAtlasMap = currentAtlas.first;
  auto &depthAtlasMap = currentAtlas.second;

  const auto &textureViewMap = currentView.first;
  const auto &depthViewMap = currentView.second;

  int w = patch.patchSize.x(), h = patch.patchSize.y();
  int xM = patch.patchMappingPos.x(), yM = patch.patchMappingPos.y();
  int xP = patch.patchPackingPos.x(), yP = patch.patchPackingPos.y();

  if (patch.patchRotation == Metadata::PatchRotation::upright) {
    for (int dy = 0; dy < h; dy++) {
      // Y
      std::copy(textureViewMap.getPlane(0).row_begin(yM + dy) + xM,
                textureViewMap.getPlane(0).row_begin(yM + dy) + (xM + w),
                textureAtlasMap.getPlane(0).row_begin(yP + dy) + xP);

      // UV
      if ((dy % 2) == 0) {
        for (int p = 1; p < 3; p++) {
          std::copy(
              textureViewMap.getPlane(p).row_begin((yM + dy) / 2) + xM / 2,
              textureViewMap.getPlane(p).row_begin((yM + dy) / 2) +
                  (xM + w) / 2,
              textureAtlasMap.getPlane(p).row_begin((yP + dy) / 2) + xP / 2);
        }
      }

      // Depth
      std::copy(depthViewMap.getPlane(0).row_begin(yM + dy) + xM,
                depthViewMap.getPlane(0).row_begin(yM + dy) + (xM + w),
                depthAtlasMap.getPlane(0).row_begin(yP + dy) + xP);
    }
  } else {
    for (int dy = 0; dy < h; dy++) {
      // Y
      std::copy(textureViewMap.getPlane(0).row_begin(yM + dy) + xM,
                textureViewMap.getPlane(0).row_begin(yM + dy) + (xM + w),
                std::make_reverse_iterator(
                    textureAtlasMap.getPlane(0).col_begin(xP + dy) + (yP + w)));

      // UV
      if ((dy % 2) == 0) {
        for (int p = 1; p < 3; p++) {
          std::copy(textureViewMap.getPlane(p).row_begin((yM + dy) / 2) +
                        xM / 2,
                    textureViewMap.getPlane(p).row_begin((yM + dy) / 2) +
                        (xM + w) / 2,
                    std::make_reverse_iterator(
                        textureAtlasMap.getPlane(p).col_begin((xP + dy) / 2) +
                        (yP + w) / 2));
        }
      }

      // Depth
      std::copy(depthViewMap.getPlane(0).row_begin(yM + dy) + xM,
                depthViewMap.getPlane(0).row_begin(yM + dy) + (xM + w),
                std::make_reverse_iterator(
                    depthAtlasMap.getPlane(0).col_begin(xP + dy) + (yP + w)));
    }
  }
}

} // namespace TMIV::AtlasConstructor
