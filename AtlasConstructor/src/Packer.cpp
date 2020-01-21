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

#include "MaxRectPiP.h"
#include <TMIV/AtlasConstructor/Packer.h>
#include <iostream>
#include <queue>
#include <stdexcept>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::AtlasConstructor {

Packer::Packer(const Json &rootNode, const Json &componentNode) {
  m_alignment = componentNode.require("Alignment").asInt();
  m_minPatchSize = componentNode.require("MinPatchSize").asInt();
  m_overlap = componentNode.require("Overlap").asInt();
  m_pip = componentNode.require("PiP").asInt() != 0;
  m_maxEntities = rootNode.require("maxEntities").asInt();
  if (m_maxEntities > 1) {
    m_EntityEncodeRange = rootNode.require("GroupBasedEncoder")
                              .require("EntityBasedAtlasConstructor")
                              .require("EntityEncodeRange")
                              .asIntVector<2>();
  }
}

void Packer::updateAggEntityMasks(ME16Frame entityMasks) {
  for (const auto &entityMask : entityMasks) {
    m_aggEntityMasks.push_back(entityMask);
  }
}

auto Packer::setMask(int vIndex, int eIndex) -> Mask {
  Mask mask(m_aggEntityMasks[vIndex].getWidth(), m_aggEntityMasks[vIndex].getHeight());
  fill(mask.getPlane(0).begin(), mask.getPlane(0).end(), uint8_t(0));
  vector<int> Indices(mask.getPlane(0).size());
  std::iota(Indices.begin(), Indices.end(), 0);
  std::for_each(Indices.begin(), Indices.end(), [&](auto i) {
    if (m_aggEntityMasks[vIndex].getPlane(0)[i] == eIndex) {
      mask.getPlane(0)[i] = uint8_t(255);
    }
  });
  return mask;
}

auto Packer::pack(const SizeVector &atlasSizes, const MaskList &masks,
                  const vector<bool> &isBasicView) -> AtlasParamsVector {

  // Check atlas size
  for (const auto &sz : atlasSizes) {
    if (((sz.x() % m_alignment) != 0) || ((sz.y() % m_alignment) != 0)) {
      throw std::runtime_error("Atlas size should be a multiple of aligment");
    }
  }

  // Mask clustering
  ClusterList clusterList;
  ClusteringMapList clusteringMap;

  for (auto viewId = 0; viewId < int(masks.size()); viewId++) {
    if (m_maxEntities > 1) {
      for (int eIndex = m_EntityEncodeRange[0]; eIndex <= m_EntityEncodeRange[1]; eIndex++) {
        // Entity Clustering
        Mask mask = setMask(viewId, eIndex);

        auto clusteringOutput = Cluster::retrieve(
            viewId, mask, static_cast<int>(clusterList.size()), isBasicView[viewId]);

        for (auto &cluster : clusteringOutput.first) {
          cluster = Cluster::setEntityId(cluster, eIndex);
        }

        move(clusteringOutput.first.begin(), clusteringOutput.first.end(),
             back_inserter(clusterList));
        clusteringMap.push_back(move(clusteringOutput.second));
        if (!clusteringOutput.first.empty()) {
          cout << "entity " << eIndex << " from view " << viewId << " results in "
               << clusteringOutput.first.size() << " patch \n";
        }
      }
    } else {
      auto clusteringOutput = Cluster::retrieve(
          viewId, masks[viewId], static_cast<int>(clusterList.size()), isBasicView[viewId]);

      move(clusteringOutput.first.begin(), clusteringOutput.first.end(),
           back_inserter(clusterList));
      clusteringMap.push_back(move(clusteringOutput.second));
    }
  }

  // Packing
  AtlasParamsVector atlasParamsVector;
  vector<MaxRectPiP> packerList;
  MaxRectPiP::Output packerOutput;

  packerList.reserve(atlasSizes.size());
  for (const auto &sz : atlasSizes) {
    packerList.emplace_back(sz.x(), sz.y(), m_alignment, m_pip);
  }

  auto comp = [&](const Cluster &p1, const Cluster &p2) -> bool {
    if (isBasicView[p1.getViewId()] != isBasicView[p2.getViewId()]) {
      return isBasicView[p2.getViewId()];
    }
    if (m_maxEntities > 1) {
      return p1.getNumActivePixels() < p2.getNumActivePixels();
    }
    return p1.getArea() < p2.getArea();
  };

  priority_queue<Cluster, vector<Cluster>, decltype(comp)> clusterToPack(comp);

  for (const auto &cluster : clusterList) {
    // modification to align the imin,jmin to even values to help renderer
    Cluster c = Cluster::align(cluster, 2);
    clusterToPack.push(c);
  }

  int pIndex = 0;
  int clusteringMap_viewId;
  while (!clusterToPack.empty()) {
    const Cluster &cluster = clusterToPack.top();
    if (m_maxEntities > 1) {
      clusteringMap_viewId =
          (cluster.getEntityId() - m_EntityEncodeRange[0]) * static_cast<int>(masks.size()) +
          cluster.getViewId();
    } else {
      clusteringMap_viewId = cluster.getViewId();
    }
    if (m_minPatchSize <= cluster.getMinSize()) {
      bool packed = false;

      for (size_t atlasId = 0; atlasId < packerList.size(); ++atlasId) {
        MaxRectPiP &packer = packerList[atlasId];

        if (packer.push(cluster, clusteringMap[clusteringMap_viewId], packerOutput)) {
          AtlasParameters p;

          p.atlasId = static_cast<uint8_t>(atlasId);
          p.viewId = static_cast<uint8_t>(cluster.getViewId());
          p.patchSizeInView = {align(cluster.width(), m_alignment),
                               align(cluster.height(), m_alignment)};
          p.posInView = {cluster.jmin(), cluster.imin()};
          p.posInAtlas = {packerOutput.x(), packerOutput.y()};
          p.rotation = packerOutput.isRotated() ? PatchRotation::rot270 : PatchRotation::none;

          auto patchOverflow =
              (p.posInView + p.patchSizeInView) - masks[cluster.getViewId()].getSize();
          if (patchOverflow.x() > 0) {
            p.posInView.x() -= patchOverflow.x();
          }
          if (patchOverflow.y() > 0) {
            p.posInView.y() -= patchOverflow.y();
          }

          if (m_maxEntities > 1) {
            p.entityId = cluster.getEntityId();
            cout << "Packing patch " << pIndex << " of entity " << *p.entityId << " from view "
                 << p.viewId << " with #active pixels " << cluster.getNumActivePixels()
                 << " in atlas " << static_cast<int>(p.atlasId) << endl;
          }

          atlasParamsVector.push_back(p);
          pIndex++;

          packed = true;
          break;
        }
      }

      if (!packed) {
        auto cc = cluster.split(clusteringMap[clusteringMap_viewId], m_overlap);

        if (m_minPatchSize <= cc.first.getMinSize()) {
          // modification to align the imin,jmin to even values to help renderer
          Cluster c = Cluster::align(cc.first, 2);
          clusterToPack.push(c);
        }

        if (m_minPatchSize <= cc.second.getMinSize()) {
          // modification to align the imin,jmin to even values to help renderer
          Cluster c = Cluster::align(cc.second, 2);
          clusterToPack.push(c);
        }
      }
    }

    clusterToPack.pop();
  }

  return atlasParamsVector;
}

} // namespace TMIV::AtlasConstructor
