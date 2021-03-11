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

#include <TMIV/Packer/Retriever.h>

namespace TMIV::Packer {

static const uint16_t ACTIVE = 65534;
static const uint16_t INVALID = 65535;

using Common::Vec2i;

namespace {
template <typename ClusterBufferType>
void addCandidate(int a, int b, std::queue<Vec2i> &candidates, ClusterBufferType &clusteringBuffer,
                  int ID, Cluster &subCluster) {
  if (clusteringBuffer(a, b) == ACTIVE) {
    candidates.push({a, b});
    subCluster.push(a, b);
    clusteringBuffer(a, b) = static_cast<uint16_t>(ID);
  }
}

template <typename ClusterBufferType>
auto mergePatches(ClusterList &clusterList, int A, int B, const Cluster &cluster,
                  std::queue<Vec2i> &&candidates, ClusterBufferType &clusteringBuffer) -> int {
  auto growSubRegion = [&](int ID) {
    Cluster subCluster(cluster.getViewId(), cluster.isBasicView(), ID, cluster.getEntityId());
    while (!candidates.empty()) {
      const auto a = candidates.front().x();
      const auto b = candidates.front().y();

      subCluster.push(a, b);
      clusteringBuffer(a, b) = static_cast<uint16_t>(ID);

      if (0 < a) {
        addCandidate(a - 1, b, candidates, clusteringBuffer, ID, subCluster);
        if (0 < b) {
          addCandidate(a - 1, b - 1, candidates, clusteringBuffer, ID, subCluster);
        }
        if (b < B - 1) {
          addCandidate(a - 1, b + 1, candidates, clusteringBuffer, ID, subCluster);
        }
      }
      if (a < A - 1) {
        addCandidate(a + 1, b, candidates, clusteringBuffer, ID, subCluster);
        if (0 < b) {
          addCandidate(a + 1, b - 1, candidates, clusteringBuffer, ID, subCluster);
        }
        if (b < B - 1) {
          addCandidate(a + 1, b + 1, candidates, clusteringBuffer, ID, subCluster);
        }
      }
      if (0 < b) {
        addCandidate(a, b - 1, candidates, clusteringBuffer, ID, subCluster);
      }
      if (b < B - 1) {
        addCandidate(a, b + 1, candidates, clusteringBuffer, ID, subCluster);
      }
      candidates.pop();
    }
    clusterList.push_back(subCluster);
  };

  const auto i_top = cluster.imin();
  const auto i_bottom = cluster.imax();
  const auto j_left = cluster.jmin();
  const auto j_right = cluster.jmax();

  auto subClusterId = cluster.getClusterId();
  // left side
  if (j_left != 0) {
    for (int i_unit = i_top; i_unit <= i_bottom; i_unit++) {
      if (clusteringBuffer(i_unit, j_left - 1) == ACTIVE) {
        candidates.push({i_unit, j_left - 1});
        growSubRegion(++subClusterId);
      }
    }
  }
  // right side
  if (j_right != B - 1) {
    for (int i_unit = i_top; i_unit <= i_bottom; i_unit++) {
      if (clusteringBuffer(i_unit, j_right + 1) == ACTIVE) {
        candidates.push({i_unit, j_right + 1});
        growSubRegion(++subClusterId);
      }
    }
  }
  // bottom side
  if (i_bottom != A - 1) {
    for (int j_unit = j_left; j_unit <= j_right; j_unit++) {
      if (clusteringBuffer(i_bottom + 1, j_unit) == ACTIVE) {
        candidates.push({i_bottom + 1, j_unit});
        growSubRegion(++subClusterId);
      }
    }
  }
  if (!cluster.isBasicView()) {
    const auto clusterId = static_cast<uint16_t>(cluster.getClusterId());
    for (int i_inter = i_top; i_inter <= i_bottom; i_inter++) {
      for (int j_inter = j_left; j_inter <= j_right; j_inter++) {
        if (clusteringBuffer(i_inter, j_inter) == ACTIVE) {
          clusteringBuffer(i_inter, j_inter) = clusterId;
        }
      }
    }
  }

  return subClusterId;
}

template <typename ClusterBufferType>
auto getInitialCandidates(ClusterBufferType &clusteringBuffer, const int A, const int B,
                          int clusterId, Cluster &cluster, const div_t &dv) -> std::queue<Vec2i> {
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
    -> std::vector<int> {

  std::vector<int> activeList{};

  for (size_t i = 0; i < maskBuffer.size(); i++) {
    if (0 < maskBuffer[i]) {
      activeList.push_back(static_cast<int>(i));
      clusteringBuffer[i] = ACTIVE;
    } else {
      clusteringBuffer[i] = INVALID;
    }
  }
  return activeList;
}

template <typename ClusterBufferType>
void updateSeedAndNumberOfActivePixels(const std::vector<int> &activeList, Cluster &cluster,
                                       const ClusterBufferType &clusteringBuffer,
                                       std::vector<int>::const_iterator &iter_seed) {
  const auto prevIter = iter_seed;
  iter_seed = find_if(iter_seed + 1, activeList.end(),
                      [&clusteringBuffer](int i) { return (clusteringBuffer[i] == ACTIVE); });
  const auto currentIter = iter_seed;
  const auto counter = static_cast<int>(distance(prevIter, currentIter));
  cluster.numActivePixels() = counter;
}

void updateOutput(const bool isBasicView, const Cluster &cluster, const int subClusterId,
                  ClusterList &clusterList, int &clusterId) {
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

auto retrieveClusters(const int viewId, const Common::Mask &maskMap, const int firstClusterId,
                      const bool isBasicView, const bool enableMerging)
    -> std::pair<ClusterList, ClusteringMap> {
  std::pair<ClusterList, ClusteringMap> out(ClusterList(),
                                            ClusteringMap(maskMap.getWidth(), maskMap.getHeight()));
  ClusterList &clusterList = out.first;
  auto &clusteringBuffer = out.second.getPlane(0);

  const auto &maskBuffer = maskMap.getPlane(0);
  const auto A = static_cast<int>(maskBuffer.m());
  const auto B = static_cast<int>(maskBuffer.n());

  const auto activeList = buildActiveList(maskBuffer, clusteringBuffer);

  // Region growing
  int clusterId = firstClusterId;
  auto iter_seed = activeList.begin();

  while (iter_seed != activeList.end()) {
    div_t dv = div(*iter_seed, B);
    Cluster cluster(viewId, isBasicView, clusterId, 0);

    cluster.push(dv.quot, dv.rem);
    clusteringBuffer(dv.quot, dv.rem) = static_cast<uint16_t>(clusterId);

    auto candidates = getInitialCandidates(clusteringBuffer, A, B, clusterId, cluster, dv);

    const auto subClusterId = [&]() {
      if (enableMerging) {
        return mergePatches(clusterList, A, B, cluster, std::move(candidates), clusteringBuffer);
      }
      return clusterId;
    }();

    updateSeedAndNumberOfActivePixels(activeList, cluster, clusteringBuffer, iter_seed);

    updateOutput(isBasicView, cluster, subClusterId, clusterList, clusterId);
  }

  return out;
}

} // namespace TMIV::Packer
