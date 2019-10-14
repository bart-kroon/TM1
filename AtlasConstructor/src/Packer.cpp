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
#include <queue>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::AtlasConstructor {

Packer::Packer(const Json & /*rootNode*/, const Json &componentNode) {
  m_alignment = componentNode.require("Alignment").asInt();
  m_minPatchSize = componentNode.require("MinPatchSize").asInt();
  m_overlap = componentNode.require("Overlap").asInt();
  m_pip = componentNode.require("PiP").asInt() != 0;
}

AtlasParametersList Packer::pack(const vector<Vec2i> &atlasSize, const MaskList &masks,
                                 const vector<uint8_t> &shouldNotBeSplit) {

  // Mask clustering
  ClusterList clusterList;
  ClusteringMapList clusteringMap;

  for (auto cameraId = 0; cameraId < int(masks.size()); cameraId++) {
    auto clusteringOutput =
        Cluster::retrieve(cameraId, masks[cameraId], static_cast<int>(clusterList.size()),
                          shouldNotBeSplit[cameraId] != 0U);

    move(clusteringOutput.first.begin(), clusteringOutput.first.end(), back_inserter(clusterList));
    clusteringMap.push_back(move(clusteringOutput.second));
  }

  // Packing
  AtlasParametersList patchList;
  vector<MaxRectPiP> packerList;
  MaxRectPiP::Output packerOutput;

  packerList.reserve(atlasSize.size());
  for (const auto &sz : atlasSize) {
    packerList.emplace_back(sz.x(), sz.y(), m_alignment, m_pip);
  }

  auto comp = [&](const Cluster &p1, const Cluster &p2) {
    if (shouldNotBeSplit[p1.getCameraId()] != shouldNotBeSplit[p2.getCameraId()]) {
      return (shouldNotBeSplit[p2.getCameraId()] != 0U);
    }
    { return (p1.getArea() < p2.getArea()); }
  };

  priority_queue<Cluster, vector<Cluster>, decltype(comp)> clusterToPack(comp);

  for (const auto &cluster : clusterList) {
    // modification to align the imin,jmin to even values to help renderer
    Cluster c = Cluster::align(cluster, 2);
    clusterToPack.push(c);
  }
  while (!clusterToPack.empty()) {
    const Cluster &cluster = clusterToPack.top();

    if (m_minPatchSize <= cluster.getMinSize()) {
      bool packed = false;

      for (size_t atlasId = 0; atlasId < packerList.size(); ++atlasId) {
        MaxRectPiP &packer = packerList[atlasId];

        if (packer.push(cluster, clusteringMap[cluster.getCameraId()], packerOutput)) {
          AtlasParameters p;

          p.atlasId = static_cast<uint8_t>(atlasId);
          p.viewId = static_cast<uint8_t>(cluster.getCameraId());
          p.patchSizeInView = {align(cluster.width(), m_alignment),
                               align(cluster.height(), m_alignment)};
          p.posInView = {cluster.jmin(), cluster.imin()};
          p.posInAtlas = {packerOutput.x(), packerOutput.y()};
          p.rotation = packerOutput.isRotated() ? PatchRotation::rot270 : PatchRotation::none;

          auto patchOverflow =
              (p.posInView + p.patchSizeInView) - masks[cluster.getCameraId()].getSize();
          if (patchOverflow.x() > 0) {
            p.posInView.x() -= patchOverflow.x();
          }
          if (patchOverflow.y() > 0) {
            p.posInView.y() -= patchOverflow.y();
          }

          patchList.push_back(p);

          packed = true;
          break;
        }
      }

      if (!packed) {
        auto cc = cluster.split(clusteringMap[cluster.getCameraId()], m_overlap);

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

  return patchList;
}

} // namespace TMIV::AtlasConstructor
