/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include <TMIV/Packer/Retriever.h>

namespace TMIV::Packer {
static const uint16_t ACTIVE = 65534;
static const uint16_t INVALID = 65535;

using Common::Vec2i;

namespace {
template <typename ClusterBufferType> class SubRegionGrower {
public:
  SubRegionGrower(ClusterList &clusterList, int32_t A, int32_t B, const Cluster &cluster,
                  std::queue<Vec2i> &candidates, ClusterBufferType &clusteringBuffer)
      : m_clusterList{clusterList}
      , m_A{A}
      , m_B{B}
      , m_cluster{cluster}
      , m_candidates{candidates}
      , m_clusteringBuffer{clusteringBuffer} {}

  auto operator()(int32_t ID) {
    Cluster subCluster(m_cluster.getViewIdx(), m_cluster.isBasicView(), m_cluster.isSemiBasicView(),
                       ID, m_cluster.getEntityId());
    while (!m_candidates.empty()) {
      const auto a = m_candidates.front().x();
      const auto b = m_candidates.front().y();

      subCluster.push(a, b);
      m_clusteringBuffer(a, b) = static_cast<uint16_t>(ID);

      if (0 < a) {
        addCandidate(a - 1, b, ID, subCluster);
        if (0 < b) {
          addCandidate(a - 1, b - 1, ID, subCluster);
        }
        if (b < m_B - 1) {
          addCandidate(a - 1, b + 1, ID, subCluster);
        }
      }
      if (a < m_A - 1) {
        addCandidate(a + 1, b, ID, subCluster);
        if (0 < b) {
          addCandidate(a + 1, b - 1, ID, subCluster);
        }
        if (b < m_B - 1) {
          addCandidate(a + 1, b + 1, ID, subCluster);
        }
      }
      if (0 < b) {
        addCandidate(a, b - 1, ID, subCluster);
      }
      if (b < m_B - 1) {
        addCandidate(a, b + 1, ID, subCluster);
      }
      m_candidates.pop();
    }
    m_clusterList.push_back(subCluster);
  }

private:
  void addCandidate(int32_t a, int32_t b, int32_t ID, Cluster &subCluster) {
    if (m_clusteringBuffer(a, b) == ACTIVE) {
      m_candidates.push({a, b});
      subCluster.push(a, b);
      m_clusteringBuffer(a, b) = static_cast<uint16_t>(ID);
    }
  }

  ClusterList &m_clusterList;
  int32_t m_A;
  int32_t m_B;
  const Cluster &m_cluster;
  std::queue<Vec2i> &m_candidates;
  ClusterBufferType &m_clusteringBuffer;
};

template <typename ClusterBufferType>
auto mergePatches(ClusterList &clusterList, std::pair<int32_t, int32_t> AB, Cluster &cluster,
                  std::queue<Vec2i> candidates, ClusterBufferType &clusteringBuffer,
                  const TMIV::Common::Mat<uint32_t> &infomationBuffer) -> int32_t {
  SubRegionGrower<ClusterBufferType> growSubRegion{clusterList, AB.first,   AB.second,
                                                   cluster,     candidates, clusteringBuffer};

  auto A = AB.first;
  auto B = AB.second;
  const auto i_top = cluster.imin();
  const auto i_bottom = cluster.imax();
  const auto j_left = cluster.jmin();
  const auto j_right = cluster.jmax();

  auto subClusterId = cluster.getClusterId();
  // left side
  if (j_left != 0) {
    for (int32_t i_unit = i_top; i_unit <= i_bottom; i_unit++) {
      if (clusteringBuffer(i_unit, j_left - 1) == ACTIVE) {
        candidates.push({i_unit, j_left - 1});
        growSubRegion(++subClusterId);
      }
    }
  }
  // right side
  if (j_right != B - 1) {
    for (int32_t i_unit = i_top; i_unit <= i_bottom; i_unit++) {
      if (clusteringBuffer(i_unit, j_right + 1) == ACTIVE) {
        candidates.push({i_unit, j_right + 1});
        growSubRegion(++subClusterId);
      }
    }
  }
  // bottom side
  if (i_bottom != A - 1) {
    for (int32_t j_unit = j_left; j_unit <= j_right; j_unit++) {
      if (clusteringBuffer(i_bottom + 1, j_unit) == ACTIVE) {
        candidates.push({i_bottom + 1, j_unit});
        growSubRegion(++subClusterId);
      }
    }
  }

  if (!cluster.isBasicView()) {
    if (!infomationBuffer.empty()) {
      cluster.calculateInformationDensityWithBuffer(clusteringBuffer, infomationBuffer);
    }
    const auto clusterId = static_cast<uint16_t>(cluster.getClusterId());
    for (int32_t i_inter = i_top; i_inter <= i_bottom; i_inter++) {
      for (int32_t j_inter = j_left; j_inter <= j_right; j_inter++) {
        if (clusteringBuffer(i_inter, j_inter) == ACTIVE) {
          if (infomationBuffer.empty() ||
              std::abs(static_cast<float>(infomationBuffer(i_inter, j_inter)) -
                       static_cast<float>(cluster.getInformationDensity())) <
                  0.2 * cluster.getInformationDensity()) {
            clusteringBuffer(i_inter, j_inter) = clusterId;
          }
        }
      }
    }
  }

  return subClusterId;
}

template <typename ClusterBufferType>
auto getInitialCandidates(ClusterBufferType &clusteringBuffer, const int32_t A, const int32_t B,
                          int32_t clusterId, Cluster &cluster, const div_t &dv)
    -> std::queue<Vec2i> {
  std::queue<Vec2i> candidates;
  candidates.push({dv.quot, dv.rem});
  auto tryAddCandidate = [&clusteringBuffer, &cluster, clusterId, &candidates](const Vec2i &p) {
    uint16_t &visitedId = clusteringBuffer(p.x(), p.y());

    if (visitedId == ACTIVE) {
      cluster.push(p.x(), p.y());
      visitedId = static_cast<uint16_t>(clusterId);
      candidates.push({p.x(), p.y()});
    }
  };

  while (!candidates.empty()) {
    const auto a = candidates.front().x();
    const auto b = candidates.front().y();

    if (0 < a) {
      tryAddCandidate({a - 1, b});

      if (0 < b) {
        tryAddCandidate({a - 1, b - 1});
      }
      if (b < B - 1) {
        tryAddCandidate({a - 1, b + 1});
      }
    }

    if (a < A - 1) {
      tryAddCandidate({a + 1, b});

      if (0 < b) {
        tryAddCandidate({a + 1, b - 1});
      }
      if (b < B - 1) {
        tryAddCandidate({a + 1, b + 1});
      }
    }

    if (0 < b) {
      tryAddCandidate({a, b - 1});
    }
    if (b < B - 1) {
      tryAddCandidate({a, b + 1});
    }

    candidates.pop();
  }

  return candidates;
}

template <typename ClusterBufferType, typename MaskBufferType>
auto buildActiveList(const MaskBufferType &maskBuffer, ClusterBufferType &clusteringBuffer)
    -> std::vector<int32_t> {
  std::vector<int32_t> activeList{};

  for (size_t i = 0; i < maskBuffer.size(); i++) {
    if (0 < maskBuffer[i]) {
      activeList.push_back(static_cast<int32_t>(i));
      clusteringBuffer[i] = ACTIVE;
    } else {
      clusteringBuffer[i] = INVALID;
    }
  }
  return activeList;
}

template <typename ClusterBufferType>
void updateSeedAndNumberOfActivePixels(const std::vector<int32_t> &activeList, Cluster &cluster,
                                       const ClusterBufferType &clusteringBuffer,
                                       std::vector<int32_t>::const_iterator &iter_seed) {
  const auto prevIter = iter_seed;
  iter_seed = find_if(iter_seed + 1, activeList.end(),
                      [&clusteringBuffer](int32_t i) { return (clusteringBuffer[i] == ACTIVE); });
  const auto currentIter = iter_seed;
  const auto counter = static_cast<int32_t>(distance(prevIter, currentIter));
  cluster.numActivePixels() = counter;
}

void updateOutput(const bool isBasicView, const Cluster &cluster, const int32_t subClusterId,
                  ClusterList &clusterList, int32_t &clusterId) {
  if (isBasicView) {
    if (!clusterList.empty()) {
      clusterList[0] = Cluster::merge(clusterList[0], cluster);
    } else {
      clusterList.push_back(cluster);
    }

  } else {
    clusterList.push_back(cluster);
    clusterId = subClusterId + 1; // Patch Merging
  }
}
} // namespace

auto retrieveClusters(const int32_t viewIdx, const Common::Frame<uint8_t> &maskMap,
                      const Common::Frame<uint32_t> &informationMap, const int32_t firstClusterId,
                      const std::pair<bool, bool> isBasicOrSemiBasicView, flags m_flags)
    -> std::pair<ClusterList, ClusteringMap> {
  std::pair<ClusterList, ClusteringMap> out(
      ClusterList(), ClusteringMap::lumaOnly({maskMap.getWidth(), maskMap.getHeight()}));
  ClusterList &clusterList = out.first;
  auto &clusteringBuffer = out.second.getPlane(0);

  const auto &maskBuffer = maskMap.getPlane(0);
  const auto &informationBuffer =
      !informationMap.empty() ? informationMap.getPlane(0) : TMIV::Common::Mat<uint32_t>();
  const auto A = static_cast<int32_t>(maskBuffer.m());
  const auto B = static_cast<int32_t>(maskBuffer.n());

  const auto activeList = buildActiveList(maskBuffer, clusteringBuffer);

  auto isBasicView = isBasicOrSemiBasicView.first;
  auto isSemiBasicView = isBasicOrSemiBasicView.second;

  // Region growing
  int32_t clusterId = firstClusterId;
  auto iter_seed = activeList.begin();
  // NOTE(FT): a basic view is packed as a single patch, hence no need for any clustering, except
  // when entities are present.
  if (isBasicView && !m_flags.multiEntity) {
    Cluster cluster(viewIdx, isBasicView, isSemiBasicView, clusterId, 0);
    for (size_t i = 0; i < maskBuffer.size(); i++) {
      if (0 < maskBuffer[i]) {
        div_t dv = div(static_cast<int32_t>(i), B);
        cluster.push(dv.quot, dv.rem);
        clusteringBuffer[i] = static_cast<uint16_t>(clusterId);
      }
    }
    clusterList.push_back(cluster);
  } else if (isSemiBasicView) {
    Cluster cluster(viewIdx, isBasicView, isSemiBasicView, clusterId, 0);
    for (size_t i = 0; i < maskBuffer.size(); i++) {
      if (0 < maskBuffer[i]) {
        div_t dv = div(static_cast<int32_t>(i), B);
        cluster.push(dv.quot, dv.rem);
        clusteringBuffer[i] = static_cast<uint16_t>(clusterId);
      }
    }
    clusterList.push_back(cluster);
  } else {
    while (iter_seed != activeList.end()) {
      div_t dv = div(*iter_seed, B);
      Cluster cluster(viewIdx, isBasicView, isSemiBasicView, clusterId, 0);

      cluster.push(dv.quot, dv.rem);
      clusteringBuffer(dv.quot, dv.rem) = static_cast<uint16_t>(clusterId);

      auto candidates = getInitialCandidates(clusteringBuffer, A, B, clusterId, cluster, dv);

      const auto subClusterId = [&]() {
        if (m_flags.enableMerging) {
          return mergePatches(clusterList, std::make_pair(A, B), cluster, std::move(candidates),
                              clusteringBuffer, informationBuffer);
        }
        return clusterId;
      }();

      updateSeedAndNumberOfActivePixels(activeList, cluster, clusteringBuffer, iter_seed);

      updateOutput(isBasicView, cluster, subClusterId, clusterList, clusterId);
    }
  }
  return out;
}

} // namespace TMIV::Packer
