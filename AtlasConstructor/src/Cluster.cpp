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
#include <queue>

namespace TMIV::AtlasConstructor {

static const std::uint16_t ACTIVE = 65534;
static const std::uint16_t INVALID = 65535;

Cluster::Cluster(int cameraId, int clusterId)
    : cameraId_(cameraId), clusterId_(clusterId) {}

void Cluster::push(int i, int j) {
  if (i < imin_) {
    imin_ = i;
  }
  if (imax_ < i) {
    imax_ = i;
  }
  if (j < jmin_) {
    jmin_ = j;
  }
  if (jmax_ < j) {
    jmax_ = j;
  }

  filling_++;
}

Cluster Cluster::align(const Cluster &c, int alignment) {
  Cluster d(c.cameraId_, c.clusterId_);

  d.imin_ = c.imin_ - (c.imin_ % alignment);
  d.imax_ = c.imax_; // modification to align the imin,jmin to even values to
                     // help renderer

  d.jmin_ = c.jmin_ - (c.jmin_ % alignment);
  d.jmax_ = c.jmax_; // modification to align the imin,jmin to even values to
                     // help renderer

  d.filling_ = c.filling_;

  return d;
}

Cluster Cluster::merge(const Cluster &c1, const Cluster &c2) {
  Cluster c(c1.cameraId_, c1.clusterId_);

  c.imin_ = std::min(c1.imin_, c2.imin_);
  c.imax_ = std::max(c1.imax_, c2.imax_);

  c.jmin_ = std::min(c1.jmin_, c2.jmin_);
  c.jmax_ = std::max(c1.jmax_, c2.jmax_);

  c.filling_ = (c1.filling_ + c2.filling_);

  return c;
}

std::pair<Cluster, Cluster> Cluster::split(const ClusteringMap &clusteringMap,
                                           int overlap) const {

  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  const Cluster &c = *this;
  Cluster c1(c.getCameraId(), c.getClusterId());
  Cluster c2(c.getCameraId(), c.getClusterId());

  if (c.width() < c.height()) {
    int imid = (c.imin() + c.imax()) / 2;
    int imid1 =
        std::min(imid + overlap, static_cast<int>(clusteringBuffer.m()) - 1);
    int imid2 = std::max(0, imid - overlap);

    for (int i = c.imin(); i < imid1; i++) {
      for (int j = c.jmin(); j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c1.push(i, j);
        }
      }
    }

    for (int i = imid2; i <= c.imax(); i++) {
      for (int j = c.jmin(); j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c2.push(i, j);
        }
      }
    }
  } else {
    int jmid = (c.jmin() + c.jmax()) / 2;
    int jmid1 =
        std::min(jmid + overlap, static_cast<int>(clusteringBuffer.n()) - 1);
    int jmid2 = std::max(0, jmid - overlap);

    for (int i = c.imin(); i <= c.imax(); i++) {
      for (int j = c.jmin(); j < jmid1; j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c1.push(i, j);
        }
      }
    }

    for (int i = c.imin(); i <= c.imax(); i++) {
      for (int j = jmid2; j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c2.push(i, j);
        }
      }
    }
  }

  return std::pair<Cluster, Cluster>(c1, c2);
}

std::pair<ClusterList, ClusteringMap>
Cluster::retrieve(int cameraId, const Common::Mask &maskMap, int firstClusterId,
                  bool shouldNotBeSplit) {

  std::pair<ClusterList, ClusteringMap> out(
      ClusterList(), ClusteringMap(maskMap.getWidth(), maskMap.getHeight()));
  ClusterList &clusterList = out.first;
  auto &clusteringBuffer = out.second.getPlane(0);

  const auto &maskBuffer = maskMap.getPlane(0);
  int A = int(maskBuffer.m());
  int B = int(maskBuffer.n());
  int S = int(maskBuffer.size());

  // Build active list
  std::vector<int> activeList;

  activeList.reserve(S);

  for (int i = 0; i < S; i++) {
    if (0 < maskBuffer[i]) {
      activeList.push_back(i);
      clusteringBuffer[i] = ACTIVE;
    } else {
      clusteringBuffer[i] = INVALID;
    }
  }

  // Region growing
  int clusterId = firstClusterId;
  auto iter_seed = activeList.begin();
  int clustered = 0;

  while (iter_seed != activeList.end()) {
    std::div_t dv = std::div(*iter_seed, B);
    Cluster cluster(cameraId, clusterId);
    std::queue<std::array<int, 2>> candidates;

    cluster.push(dv.quot, dv.rem);
    candidates.push({dv.quot, dv.rem});
    clusteringBuffer(dv.quot, dv.rem) = static_cast<uint16_t>(clusterId);

    auto tryAddCandidate = [&](int a, int b) {
      std::uint16_t &visitedId = clusteringBuffer(a, b);

      if (visitedId == ACTIVE) {
        cluster.push(a, b);
        visitedId = uint16_t(clusterId);
        candidates.push({a, b});
      }
    };

    while (!candidates.empty()) {
      const std::array<int, 2> &current = candidates.front();
      int a = current[0];
      int b = current[1];

      if (0 < a) {
        tryAddCandidate(a - 1, b);

        if (0 < b) {
          tryAddCandidate(a - 1, b - 1);
        }
        if (b < B - 1) {
          tryAddCandidate(a - 1, b + 1);
        }
      }

      if (a < A - 1) {
        tryAddCandidate(a + 1, b);

        if (0 < b) {
          tryAddCandidate(a + 1, b - 1);
        }
        if (b < B - 1) {
          tryAddCandidate(a + 1, b + 1);
        }
      }

      if (0 < b) {
        tryAddCandidate(a, b - 1);
      }
      if (b < B - 1) {
        tryAddCandidate(a, b + 1);
      }

      candidates.pop();
    }

    // Updating output
    if (shouldNotBeSplit) {
      if (!clusterList.empty()) {
        clusterList[0] = Cluster::merge(clusterList[0], cluster);
      } else {
        clusterList.push_back(cluster);
      }

      clustered = clusterList[0].getFilling();
    } else {
      clustered += cluster.getFilling();

      clusterList.push_back(cluster);
      clusterId++;
    }

    // Update seed
    iter_seed = std::find_if(
        iter_seed + 1, activeList.end(),
        [&clusteringBuffer](int i) { return (clusteringBuffer[i] == ACTIVE); });
  }

  return out;
}

} // namespace TMIV::AtlasConstructor
