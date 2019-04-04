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

#include "MaxRectPiP.h"
#include <TMIV/AtlasConstructor/Packer.h>
#include <queue>

namespace TMIV::AtlasConstructor {

Packer::Packer(const Common::Json &node) {
  if (auto subnode = node.optional("Alignment"))
    m_alignment = subnode.asInt();

  if (auto subnode = node.optional("MinPatchSize"))
    m_minPatchSize = subnode.asInt();

  if (auto subnode = node.optional("PiP"))
    m_pip = subnode.asBool();
}

Metadata::PatchParameterList
Packer::doPacking(const std::vector<Vec2i> &atlasSize, const MaskList &masks,
                  const std::vector<std::uint8_t> &shouldNotBeSplit) {
  
  // Mask clustering
  ClusterList clusterList;
  ClusteringMapList clusteringMap;
  
  for (auto cameraId = 0u; cameraId < masks.size(); cameraId++) {
    auto clusteringOutput =
        Cluster::retrieve(cameraId, masks[cameraId], (int) clusterList.size(),
                          shouldNotBeSplit[cameraId]);

    std::move(clusteringOutput.first.begin(), clusteringOutput.first.end(),
              std::back_inserter(clusterList));
    clusteringMap.push_back(std::move(clusteringOutput.second));
  }
  
  // Packing
  PatchParameterList patchList;
  std::vector<MaxRectPiP> packerList;
  MaxRectPiP::Output packerOutput;

  for (const auto &sz : atlasSize)
    packerList.push_back(MaxRectPiP(sz.x(), sz.y(), m_alignment, m_pip));
  
  auto comp = [](const Cluster &p1, const Cluster &p2) {
    return (p1.getArea() < p2.getArea());
  };
  std::priority_queue<Cluster, std::vector<Cluster>, decltype(comp)>
      clusterToPack(comp);

  for (const auto &cluster : clusterList)
    clusterToPack.push(cluster);
  
  while (!clusterToPack.empty()) {
    const Cluster &cluster = clusterToPack.top();

    if (m_minPatchSize < cluster.getFilling()) {
      bool packed = false;
           
      for (auto atlasId = 0u; atlasId < packerList.size(); atlasId++) {
        MaxRectPiP &packer = packerList[atlasId];
          
        if (packer.push(cluster, clusteringMap[cluster.getCameraId()],
                        packerOutput)) {
          Metadata::PatchParameters p;
          
          p.atlasId = atlasId;
          p.virtualCameraId = cluster.getCameraId();
          p.patchSize = {Common::align(cluster.width(), m_alignment),
                         Common::align(cluster.height(), m_alignment)};
          p.patchMappingPos = {cluster.jmin(), cluster.imin()};
          p.patchPackingPos = {packerOutput.x(), packerOutput.y()};
          p.patchRotation = packerOutput.isRotated()
                                ? Metadata::PatchRotation::ccw
                                : Metadata::PatchRotation::upright;

          patchList.push_back(std::move(p));

          packed = true;
          break;
        }
		
      }
      
      if (!packed) {
        auto cc = cluster.split(clusteringMap[cluster.getCameraId()]);

        if (m_minPatchSize <= cc.first.getFilling())
          clusterToPack.push(std::move(cc.first));

        if (m_minPatchSize <= cc.second.getFilling())
          clusterToPack.push(std::move(cc.second));

      }
    }

    clusterToPack.pop();
  }
 
  return patchList;
}

} // namespace TMIV::AtlasConstructor
