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

using namespace std;
using namespace TMIV::Common;

namespace TMIV::AtlasConstructor {
static const uint16_t ACTIVE = 65534;
static const uint16_t INVALID = 65535;

Cluster::Cluster(int viewId, int clusterId) : viewId_(viewId), clusterId_(clusterId) {}
Cluster::Cluster(int viewId, int clusterId, int entityId)
    : viewId_(viewId), clusterId_(clusterId), entityId_(entityId) {}

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

auto Cluster::setEntityId(Cluster &c, int entityId) -> Cluster {
  Cluster d(c.viewId_, c.clusterId_, entityId);
  d.imin_ = c.imin_;
  d.imax_ = c.imax_;
  d.jmin_ = c.jmin_;
  d.jmax_ = c.jmax_;
  d.filling_ = c.filling_;
  return d;
}

auto Cluster::align(const Cluster &c, int alignment) -> Cluster {
  Cluster d(c.viewId_, c.clusterId_, c.entityId_);

  d.imin_ = c.imin_ - (c.imin_ % alignment);
  d.imax_ = c.imax_; // modification to align the imin,jmin to even values to
                     // help renderer

  d.jmin_ = c.jmin_ - (c.jmin_ % alignment);
  d.jmax_ = c.jmax_; // modification to align the imin,jmin to even values to
                     // help renderer

  d.filling_ = c.filling_;

  return d;
}

auto Cluster::merge(const Cluster &c1, const Cluster &c2) -> Cluster {
  Cluster c(c1.viewId_, c1.clusterId_, c1.entityId_);

  c.imin_ = min(c1.imin_, c2.imin_);
  c.imax_ = max(c1.imax_, c2.imax_);

  c.jmin_ = min(c1.jmin_, c2.jmin_);
  c.jmax_ = max(c1.jmax_, c2.jmax_);

  c.filling_ = (c1.filling_ + c2.filling_);

  return c;
}

auto Cluster::split(const ClusteringMap &clusteringMap, int overlap) const
    -> pair<Cluster, Cluster> {

  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  const Cluster &c = *this;
  Cluster c1(c.getViewId(), c.getClusterId(), c.getEntityId());
  Cluster c2(c.getViewId(), c.getClusterId(), c.getEntityId());

  if (c.width() < c.height()) {
    int imid = (c.imin() + c.imax()) / 2;
    int imid1 = min(imid + overlap, static_cast<int>(clusteringBuffer.m()) - 1);
    int imid2 = max(0, imid - overlap);

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
    int jmid1 = min(jmid + overlap, static_cast<int>(clusteringBuffer.n()) - 1);
    int jmid2 = max(0, jmid - overlap);

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

  return pair<Cluster, Cluster>(c1, c2);
}

auto Cluster::retrieve(int viewId, const Mask &maskMap, int firstClusterId, bool shouldNotBeSplit)
    -> pair<ClusterList, ClusteringMap> {

  pair<ClusterList, ClusteringMap> out(ClusterList(),
                                       ClusteringMap(maskMap.getWidth(), maskMap.getHeight()));
  ClusterList &clusterList = out.first;
  auto &clusteringBuffer = out.second.getPlane(0);

  const auto &maskBuffer = maskMap.getPlane(0);
  int A = int(maskBuffer.m());
  int B = int(maskBuffer.n());
  int S = int(maskBuffer.size());

  // Build active list
  vector<int> activeList;

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
    div_t dv = div(*iter_seed, B);
    Cluster cluster(viewId, clusterId);
    queue<array<int, 2>> candidates;

    cluster.push(dv.quot, dv.rem);
    candidates.push({dv.quot, dv.rem});
    clusteringBuffer(dv.quot, dv.rem) = static_cast<uint16_t>(clusterId);

    auto tryAddCandidate = [&](int a, int b) {
      uint16_t &visitedId = clusteringBuffer(a, b);

      if (visitedId == ACTIVE) {
        cluster.push(a, b);
        visitedId = uint16_t(clusterId);
        candidates.push({a, b});
      }
    };

    while (!candidates.empty()) {
      const array<int, 2> &current = candidates.front();
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
    iter_seed = find_if(iter_seed + 1, activeList.end(),
                        [&clusteringBuffer](int i) { return (clusteringBuffer[i] == ACTIVE); });
  }

  return out;
}

} // namespace TMIV::AtlasConstructor
