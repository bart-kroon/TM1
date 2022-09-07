/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#ifndef TMIV_PACKER_PACKER_H
#define TMIV_PACKER_PACKER_H

#include "Cluster.h"
#include "IPacker.h"

#include <TMIV/Common/Json.h>
#include <TMIV/MivBitstream/Tile.h>

#include <tuple>

namespace TMIV::Packer {
class MaxRectPiP;

class Packer : public IPacker {
  enum SORTING_METHOD { AREA_DESCENDING = 0, VIEW_ID_ASCENDING = 1 };

public:
  Packer(const Common::Json & /*unused*/, const Common::Json & /*componentNode*/);
  Packer(const Packer &) = delete;
  Packer(Packer &&) = default;
  auto operator=(const Packer &) -> Packer & = delete;
  auto operator=(Packer &&) -> Packer & = default;
  ~Packer() override;

  void initialize(std::vector<std::vector<MivBitstream::TilePartition>> tileSizes) override;

  void initialize(const std::vector<Common::SizeVector> &atlasSizes, int32_t blockSize) override;
  auto pack(const std::vector<Common::SizeVector> &atlasSize,
            const Common::FrameList<uint8_t> &masks,
            const MivBitstream::ViewParamsList &viewParamsList, int32_t blockSize)
      -> MivBitstream::PatchParamsList override;
  void
  updateAggregatedEntityMasks(const std::vector<Common::FrameList<uint8_t>> &entityMasks) override;

private:
  int32_t m_minPatchSize{};
  int32_t m_overlap{};
  bool m_pip{};
  bool m_enableMerging{};
  bool m_prioritizeSSI{};
  SORTING_METHOD m_sortingMethod{};
  bool m_enableRecursiveSplit{true};
  int32_t m_maxEntityId{0};
  std::vector<Common::FrameList<uint8_t>> m_aggregatedEntityMasks{};
  Common::Vec2i m_entityEncodeRange;

  std::vector<std::vector<MivBitstream::TilePartition>> m_tileList;
  std::vector<std::vector<MaxRectPiP>> m_packerList;

  auto computeClusters(const Common::FrameList<uint8_t> &masks,
                       const MivBitstream::ViewParamsList &viewParamsList)
      -> std::tuple<ClusterList, ClusteringMapList, std::vector<int32_t>>;
  auto computeClusterToPack(const MivBitstream::ViewParamsList &viewParamsList, int32_t m_blockSize,
                            ClusterList &clusterList, const ClusteringMapList &clusteringMap) const;
  [[nodiscard]] auto getViewId(const Cluster &cluster,
                               const std::vector<int32_t> &clusteringMapIndex) const -> int32_t;
  void ifEntityOrBasic(const Cluster &cluster) const;
};

} // namespace TMIV::Packer

#endif
