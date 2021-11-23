/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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
#include <TMIV/Packer/Retriever.h>

#include "MaxRectPiP.h"

#include <iostream>
#include <queue>
#include <stdexcept>

namespace TMIV::Packer {
namespace {
void checkAtlasSize(const Common::SizeVector &atlasSizes, const int32_t blockSize) {
  for (const auto &sz : atlasSizes) {
    if (((sz.x() % blockSize) != 0) || ((sz.y() % blockSize) != 0)) {
      throw std::runtime_error("Atlas size should be a multiple of blocksize");
    }
  }
}

void adaptPatchParamsToMask(MivBitstream::PatchParams &p, int32_t maskWidth, int32_t maskHeight) {
  if (p.atlasPatch3dOffsetU() + p.atlasPatch3dSizeU() > maskWidth) {
    if (p.atlasPatch3dSizeU() <= maskWidth) {
      p.atlasPatch3dOffsetU(maskWidth - p.atlasPatch3dSizeU());
    } else {
      p.atlasPatch3dOffsetU(0);
      p.atlasPatch3dSizeU(maskWidth);
    }
  }
  if (p.atlasPatch3dOffsetV() + p.atlasPatch3dSizeV() > maskHeight) {
    if (p.atlasPatch3dSizeV() <= maskHeight) {
      p.atlasPatch3dOffsetV(maskHeight - p.atlasPatch3dSizeV());
    } else {
      p.atlasPatch3dOffsetV(0);
      p.atlasPatch3dSizeV(maskHeight);
    }
  }
}

} // namespace
Packer::Packer(const Common::Json &rootNode, const Common::Json &componentNode) {
  m_minPatchSize = componentNode.require("MinPatchSize").as<int32_t>();
  m_overlap = componentNode.require("Overlap").as<int32_t>();
  m_pip = componentNode.require("PiP").as<int32_t>() != 0;
  m_enableMerging = componentNode.require("enableMerging").as<bool>();
  switch (auto sortingMethod = componentNode.require("sortingMethod").as<int32_t>()) {
  case 0:
    m_sortingMethod = AREA_DESCENDING;
    break;
  case 1:
    m_sortingMethod = VIEW_ID_ASCENDING;
    break;
  default:
    throw std::runtime_error(fmt::format("Sorting method {} is not available", sortingMethod));
  }
  m_enableRecursiveSplit = componentNode.require("enableRecursiveSplit").as<bool>();

  if (const auto node = rootNode.optional("maxEntityId")) {
    m_maxEntityId = node.as<int32_t>();
  }
  if (m_maxEntityId > 0) {
    m_entityEncodeRange = rootNode.require("EntityEncodeRange").asVec<int32_t, 2>();
  }
}

Packer::~Packer() = default;

void Packer::updateAggregatedEntityMasks(
    const std::vector<Common::FrameList<uint8_t>> &entityMasks) {
  for (const auto &entityMask : entityMasks) {
    m_aggregatedEntityMasks.push_back(entityMask);
  }
}

auto Packer::computeClusterToPack(const MivBitstream::ViewParamsList &viewParamsList,
                                  const int32_t m_blockSize, ClusterList &clusterList,
                                  const ClusteringMapList &clusteringMap) const {
  auto comp = [this, &viewParamsList](const Cluster &p1, const Cluster &p2) -> bool {
    if (viewParamsList[p1.getViewIdx()].isBasicView !=
        viewParamsList[p2.getViewIdx()].isBasicView) {
      return viewParamsList[p2.getViewIdx()].isBasicView;
    }
    // NOTE(FT): added for packing patches from MPI ==> reading in writePatchInAtlas is done in
    // increasing mpiLayerId order
    if (m_sortingMethod == AREA_DESCENDING) {
      if (p1.getArea() != p2.getArea()) {
        return p1.getArea() < p2.getArea();
      }
    } else if (m_sortingMethod == VIEW_ID_ASCENDING) {
      if (p1.getViewIdx() != p2.getViewIdx()) {
        return p1.getViewIdx() > p2.getViewIdx();
      }
    }
    // NOTE(BK): Stable ordering
    return p1.getClusterId() > p2.getClusterId();
  };

  std::priority_queue<Cluster, std::vector<Cluster>, decltype(comp)> clusterToPack(comp);

  std::vector<Cluster> out{};
  for (const auto &cluster : clusterList) {
    if (m_maxEntityId > 0 || cluster.isBasicView()) {
      out.push_back(cluster);
    } else {
      if (m_enableRecursiveSplit) {
        cluster.recursiveSplit(clusteringMap[cluster.getViewIdx()], out, m_blockSize,
                               m_minPatchSize);
      } else {
        out.push_back(cluster);
      }
    }
  }

  for (const auto &cluster : out) {
    // modification to align the imin,jmin to even values to help renderer
    Cluster c = Cluster::align(cluster, 2);
    clusterToPack.push(c);
  }
  return clusterToPack;
}

void Packer::initialize(const Common::SizeVector &atlasSizes, const int32_t blockSize) {
  m_packerList.clear();
  m_packerList.reserve(atlasSizes.size());
  for (const auto &sz : atlasSizes) {
    m_packerList.emplace_back(sz.x(), sz.y(), blockSize, m_pip);
  }
}

auto Packer::pack(const Common::SizeVector &atlasSizes, const Common::FrameList<uint8_t> &masks,
                  const MivBitstream::ViewParamsList &viewParamsList, const int32_t m_blockSize)
    -> MivBitstream::PatchParamsList {
  checkAtlasSize(atlasSizes, m_blockSize);

  auto [clusterList, clusteringMap, clusteringMapIndex] = computeClusters(masks, viewParamsList);

  auto clusterToPack =
      computeClusterToPack(viewParamsList, m_blockSize, clusterList, clusteringMap);

  // Packing
  MivBitstream::PatchParamsList atlasParamsVector{};
  MaxRectPiP::Output packerOutput{};

  int32_t patchIdx = 0;
  int32_t clusteringMap_viewId = 0;
  while (!clusterToPack.empty()) {
    const Cluster &cluster = clusterToPack.top();

    if (m_maxEntityId > 0) {
      clusteringMap_viewId = clusteringMapIndex[cluster.getClusterId()];
    } else {
      clusteringMap_viewId = cluster.getViewIdx();
    }

    if (m_minPatchSize * m_minPatchSize <= cluster.getArea()) {
      bool packed = false;

      for (size_t atlasId = 0; atlasId < m_packerList.size(); ++atlasId) {
        MaxRectPiP &packer = m_packerList[atlasId];

        if (packer.push(cluster, clusteringMap[clusteringMap_viewId], packerOutput)) {
          MivBitstream::PatchParams p;

          p.atlasId(MivBitstream::AtlasId{static_cast<uint8_t>(atlasId)});
          p.atlasPatchProjectionId(viewParamsList[cluster.getViewIdx()].viewId);
          p.atlasPatch2dPosX(packerOutput.x());
          p.atlasPatch2dPosY(packerOutput.y());
          p.atlasPatch3dOffsetU(cluster.jmin());
          p.atlasPatch3dOffsetV(cluster.imin());
          p.atlasPatchOrientationIndex(packerOutput.isRotated()
                                           ? MivBitstream::FlexiblePatchOrientation::FPO_ROT270
                                           : MivBitstream::FlexiblePatchOrientation::FPO_NULL);
          p.atlasPatch3dSizeU(Common::align(cluster.width(), m_blockSize));
          p.atlasPatch3dSizeV(Common::align(cluster.height(), m_blockSize));

          adaptPatchParamsToMask(p, masks[cluster.getViewIdx()].getWidth(),
                                 masks[cluster.getViewIdx()].getHeight());

          if (m_maxEntityId > 0) {
            p.atlasPatchEntityId(cluster.getEntityId());
            std::cout << "Packing patch " << patchIdx << " of entity " << p.atlasPatchEntityId()
                      << " from view " << p.atlasPatchProjectionId() << " with #active pixels "
                      << cluster.getNumActivePixels() << " in atlas " << p.atlasId() << std::endl;
          }

          atlasParamsVector.push_back(p);
          patchIdx++;

          packed = true;
          break;
        }
      }

      if (!packed) {
        if (m_maxEntityId > 0) {
          std::cout << "Spliting cluster " << cluster.getClusterId() << std::endl;
        }
        if (cluster.isBasicView()) {
          throw std::runtime_error("Failed to pack basic view");
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

auto Packer::computeClusters(const Common::FrameList<uint8_t> &masks,
                             const MivBitstream::ViewParamsList &viewParamsList)
    -> std::tuple<ClusterList, ClusteringMapList, std::vector<int32_t>> {
  ClusterList clusterList{};
  ClusteringMapList clusteringMap{};
  std::vector<int32_t> clusteringMapIndex{};
  int32_t index = 0;
  for (auto viewIdx = 0; viewIdx < static_cast<int32_t>(masks.size()); viewIdx++) {
    if (m_maxEntityId > 0) {
      for (int32_t entityId = m_entityEncodeRange[0]; entityId < m_entityEncodeRange[1];
           entityId++) {
        // Entity clustering
        Common::Frame<uint8_t> mask =
            m_aggregatedEntityMasks[entityId - m_entityEncodeRange[0]][viewIdx];

        auto clusteringOutput = retrieveClusters(
            viewIdx, mask, static_cast<int32_t>(clusterList.size()),
            viewParamsList[viewIdx].isBasicView, m_enableMerging, m_maxEntityId > 0);

        for (auto &cluster : clusteringOutput.first) {
          cluster = Cluster::setEntityId(cluster, entityId);
        }

        std::move(clusteringOutput.first.begin(), clusteringOutput.first.end(),
                  back_inserter(clusterList));
        clusteringMap.push_back(std::move(clusteringOutput.second));

        for (size_t i = 0; i < clusteringOutput.first.size(); i++) {
          clusteringMapIndex.push_back(index);
        }

        if (!clusteringOutput.first.empty()) {
          std::cout << "entity " << entityId << " from view " << viewIdx << " results in "
                    << clusteringOutput.first.size() << " patches\n";
        }
        ++index;
      }
    } else {
      auto clusteringOutput =
          retrieveClusters(viewIdx, masks[viewIdx], static_cast<int32_t>(clusterList.size()),
                           viewParamsList[viewIdx].isBasicView, m_enableMerging, m_maxEntityId > 0);

      std::move(clusteringOutput.first.begin(), clusteringOutput.first.end(),
                back_inserter(clusterList));
      clusteringMap.push_back(std::move(clusteringOutput.second));
    }
  }
  if (m_maxEntityId > 0) {
    std::cout << "clusteringMap size = " << clusteringMap.size()
              << " with total # clusters = " << clusteringMapIndex.size() << std::endl;
  }
  return {clusterList, clusteringMap, clusteringMapIndex};
}
} // namespace TMIV::Packer
