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

#include <TMIV/Packer/Packer.h>

#include "MaxRectPiP.h"

#include <iostream>
#include <queue>
#include <stdexcept>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Packer {
Packer::Packer(const Json &rootNode, const Json &componentNode) {
  //m_blockSize = rootNode.require("blockSize").asInt();
  m_minPatchSize = componentNode.require("MinPatchSize").asInt();
  m_overlap = componentNode.require("Overlap").asInt();
  m_pip = componentNode.require("PiP").asInt() != 0;
  m_maxEntities = rootNode.require("maxEntities").asInt();
  if (m_maxEntities > 1) {
    m_entityEncodeRange =
        rootNode.require("GroupBasedEncoder").require("EntityEncodeRange").asIntVector<2>();
  }
}

void Packer::updateAggregatedEntityMasks(const vector<MaskList> &entityMasks) {
  for (const auto &entityMask : entityMasks) {
    m_aggregatedEntityMasks.push_back(entityMask);
  }
}

auto Packer::pack(const SizeVector &atlasSizes, const MaskList &masks,
                  const vector<bool> &isBasicView, const int m_blockSize, const int m_alignment)
    -> PatchParamsList {
  // Check atlas size
  for (const auto &sz : atlasSizes) {
    if (((sz.x() % m_alignment) != 0) || ((sz.y() % m_alignment) != 0)) {
      throw std::runtime_error("Atlas size should be a multiple of aligment");
    }
  }

  // Mask clustering
  ClusterList clusterList;
  ClusteringMapList clusteringMap;
  vector<int> clusteringMapIndex;
  int index = 0;
  for (auto viewId = 0; viewId < int(masks.size()); viewId++) {
    if (m_maxEntities > 1) {
      for (int entityId = m_entityEncodeRange[0]; entityId < m_entityEncodeRange[1]; entityId++) {
        // Entity clustering
        Mask mask = m_aggregatedEntityMasks[entityId - m_entityEncodeRange[0]][viewId];

        auto clusteringOutput = Cluster::retrieve(
            viewId, mask, static_cast<int>(clusterList.size()), isBasicView[viewId]);

        for (auto &cluster : clusteringOutput.first) {
          cluster = Cluster::setEntityId(cluster, entityId);
        }

        move(clusteringOutput.first.begin(), clusteringOutput.first.end(),
             back_inserter(clusterList));
        clusteringMap.push_back(move(clusteringOutput.second));

        for (size_t i = 0; i < clusteringOutput.first.size(); i++) {
          clusteringMapIndex.push_back(index);
        }

        if (!clusteringOutput.first.empty()) {
          cout << "entity " << entityId << " from view " << viewId << " results in "
               << clusteringOutput.first.size() << " patches\n";
        }
        ++index;
      }
    } else {
      auto clusteringOutput = Cluster::retrieve(
          viewId, masks[viewId], static_cast<int>(clusterList.size()), isBasicView[viewId]);

      move(clusteringOutput.first.begin(), clusteringOutput.first.end(),
           back_inserter(clusterList));
      clusteringMap.push_back(move(clusteringOutput.second));
    }
  }
  if (m_maxEntities > 1) {
    cout << "clusteringMap size = " << clusteringMap.size()
         << " with total # clusters = " << clusteringMapIndex.size() << endl;
  }

  // Packing
  PatchParamsList atlasParamsVector;
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
    return p1.getArea() < p2.getArea();
  };

  priority_queue<Cluster, vector<Cluster>, decltype(comp)> clusterToPack(comp);

  std::vector<Cluster> out;
  for (const auto &cluster : clusterList) {
    if (m_maxEntities > 1) {
      out.push_back(cluster);
    } else {
      cluster.recursiveSplit(clusteringMap[cluster.getViewId()], out, m_alignment, m_minPatchSize);
    }
  }

  for (const auto &cluster : out) {
    // modification to align the imin,jmin to even values to help renderer
    Cluster c = Cluster::align(cluster, 2);
    clusterToPack.push(c);
  }

  int patchId = 0;
  int clusteringMap_viewId = 0;
  while (!clusterToPack.empty()) {
    const Cluster &cluster = clusterToPack.top();

    if (m_maxEntities > 1) {
      clusteringMap_viewId = clusteringMapIndex[cluster.getClusterId()];
    } else {
      clusteringMap_viewId = cluster.getViewId();
    }

    if (m_minPatchSize * m_minPatchSize <= cluster.getArea()) {
      bool packed = false;

      for (size_t atlasId = 0; atlasId < packerList.size(); ++atlasId) {
        MaxRectPiP &packer = packerList[atlasId];

        if (packer.push(cluster, clusteringMap[clusteringMap_viewId], packerOutput)) {
          PatchParams p;

          p.vuhAtlasId = static_cast<uint8_t>(atlasId);

          p.pduViewId(static_cast<uint16_t>(cluster.getViewId()))
              .pduViewSize(
                  {align(cluster.width(), m_blockSize), align(cluster.height(), m_blockSize)})
              .pduViewPos({cluster.jmin(), cluster.imin()})
              .pdu2dPos({packerOutput.x(), packerOutput.y()});

          p.pduOrientationIndex(packerOutput.isRotated() ? FlexiblePatchOrientation::FPO_ROT270
                                                         : FlexiblePatchOrientation::FPO_NULL);

          auto patchOverflow =
              (p.pduViewPos() + p.pduViewSize()) - masks[cluster.getViewId()].getSize();
          if (patchOverflow.x() > 0) {
            p.pduViewPos({p.pduViewPos().x() - patchOverflow.x(), p.pduViewPos().y()});
          }
          if (patchOverflow.y() > 0) {
            p.pduViewPos({p.pduViewPos().x(), p.pduViewPos().y() - patchOverflow.y()});
          }

          if (m_maxEntities > 1) {
            p.pduEntityId(cluster.getEntityId());
            cout << "Packing patch " << patchId << " of entity " << *p.pduEntityId()
                 << " from view " << p.pduViewId() << " with #active pixels "
                 << cluster.getNumActivePixels() << " in atlas " << static_cast<int>(p.vuhAtlasId)
                 << endl;
          }

          atlasParamsVector.push_back(p);
          patchId++;

          packed = true;
          break;
        }
      }

      if (!packed) {
        if (m_maxEntities > 1) {
          cout << "Spliting cluster " << cluster.getClusterId() << endl;
        }
        auto cc = cluster.split(clusteringMap[clusteringMap_viewId], m_overlap);

        if (m_minPatchSize * m_minPatchSize <= cc.first.getArea()) {
          // modification to align the imin,jmin to even values to help renderer
          Cluster c = Cluster::align(cc.first, 2);
          clusterToPack.push(c);
          clusteringMapIndex.push_back(clusteringMap_viewId);
        }

        if (m_minPatchSize * m_minPatchSize <= cc.second.getArea()) {
          // modification to align the imin,jmin to even values to help renderer
          Cluster c = Cluster::align(cc.second, 2);
          clusterToPack.push(c);
          clusteringMapIndex.push_back(clusteringMap_viewId);
        }
      }
    }

    clusterToPack.pop();
  }

  return atlasParamsVector;
}
} // namespace TMIV::Packer
