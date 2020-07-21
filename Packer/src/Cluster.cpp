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

#include "Cluster.h"

auto roundToAlignment(int val, int alignment) -> int { return ((val) / alignment + 1); }

using namespace std;
using namespace TMIV::Common;

namespace TMIV::Packer {
static const uint16_t ACTIVE = 65534;
static const uint16_t INVALID = 65535;

Cluster::Cluster(int viewId, bool isBasicView, int clusterId, int entityId)
    : viewId_(viewId), m_isBasicView{isBasicView}, clusterId_(clusterId), entityId_(entityId) {}

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
  Cluster d(c.viewId_, c.isBasicView(), c.clusterId_, entityId);
  d.imin_ = c.imin_;
  d.imax_ = c.imax_;
  d.jmin_ = c.jmin_;
  d.jmax_ = c.jmax_;
  d.numActivePixels_ = c.numActivePixels_;
  d.filling_ = c.filling_;
  return d;
}

auto Cluster::align(const Cluster &c, int alignment) -> Cluster {
  Cluster d(c.viewId_, c.isBasicView(), c.clusterId_, c.entityId_);

  d.imin_ = c.imin_ - (c.imin_ % alignment);
  d.imax_ = c.imax_; // modification to align the imin,jmin to even values to
                     // help renderer

  d.jmin_ = c.jmin_ - (c.jmin_ % alignment);
  d.jmax_ = c.jmax_; // modification to align the imin,jmin to even values to
                     // help renderer

  d.numActivePixels_ = c.numActivePixels_;
  d.filling_ = c.filling_;

  return d;
}

auto Cluster::merge(const Cluster &c1, const Cluster &c2) -> Cluster {
  assert(!c1.isBasicView() && !c2.isBasicView());
  Cluster c(c1.viewId_, false, c1.clusterId_, c1.entityId_);

  c.imin_ = min(c1.imin_, c2.imin_);
  c.imax_ = max(c1.imax_, c2.imax_);

  c.jmin_ = min(c1.jmin_, c2.jmin_);
  c.jmax_ = max(c1.jmax_, c2.jmax_);

  c.numActivePixels_ = c1.numActivePixels_ + c2.numActivePixels_;
  c.filling_ = (c1.filling_ + c2.filling_);

  return c;
}

auto Cluster::splitLPatchHorizontally(const ClusteringMap &clusteringMap, vector<Cluster> &out,
                                      int alignment, int minPatchSize,
                                      const array<deque<int>, 2> &min_w_agg,
                                      const array<deque<int>, 2> &max_w_agg) const -> bool {
  double splitThresholdL = 0.9;

  const Cluster &c = (*this);
  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  int H = c.height();
  int W = c.width();

  int alignedImsize = roundToAlignment(W, alignment) * roundToAlignment(H, alignment);

  int minArea = alignedImsize;
  int bestSplitPos = 0;
  for (int h = minPatchSize; h < H - minPatchSize; h++) {
    int currArea = roundToAlignment(h + 1, alignment) *
                       roundToAlignment(max_w_agg[0][h] - min_w_agg[0][h], alignment) +
                   roundToAlignment(H - h - 1, alignment) *
                       roundToAlignment(max_w_agg[1][h] - min_w_agg[1][h], alignment);

    if (minArea > currArea) {
      minArea = currArea;
      bestSplitPos = h;
    }
  }

  if ((bestSplitPos != 0) && double(minArea) / alignedImsize < splitThresholdL) {
    Cluster c1(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());
    Cluster c2(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());

    for (int i = c.imin(); i < c.imin() + bestSplitPos + 1; i++) {
      for (int j = c.jmin(); j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c1.push(i, j);
        }
      }
    }
    for (int i = c.imin() + bestSplitPos - 1; i <= c.imax(); i++) {
      for (int j = c.jmin(); j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c2.push(i, j);
        }
      }
    }

    c1.recursiveSplit(clusteringMap, out, alignment, minPatchSize);
    c2.recursiveSplit(clusteringMap, out, alignment, minPatchSize);

    return true;
  }
  return false;
}

auto Cluster::splitCPatchVertically(const ClusteringMap &clusteringMap, vector<Cluster> &out,
                                    int alignment, int minPatchSize) const -> bool {
  double splitThresholdC = 0.3;

  const Cluster &c = (*this);
  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  int H = c.height();
  int W = c.width();

  int numOfEmptyBlocks = 0;
  int numOfNonEmptyBlocks = 0;

  for (int h = 0; h < H; h += alignment) {
    for (int w = 0; w < W; w += alignment) {
      bool isEmpty = true;

      for (int hh = h; hh < min(h + alignment, H); hh++) {
        int i = hh + c.imin();
        for (int ww = w; ww < min(w + alignment, W); ww++) {
          int j = ww + c.jmin();

          if (clusteringBuffer(i, j) == c.getClusterId()) {
            isEmpty = false;
            break;
          }
        } // ww
        if (!isEmpty) {
          break;
        }
      } // hh
      if (isEmpty) {
        numOfEmptyBlocks++;
      } else {
        numOfNonEmptyBlocks++;
      }
    } // w
  }   // h

  if (double(numOfNonEmptyBlocks) / (numOfEmptyBlocks + numOfNonEmptyBlocks) < splitThresholdC) {
    int bestSplitPos = roundToAlignment(W, alignment);

    Cluster c1(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());
    Cluster c2(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());

    for (int i = c.imin(); i <= c.imax(); i++) {
      for (int j = c.jmin(); j < c.jmin() + bestSplitPos + 1; j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c1.push(i, j);
        }
      }
    }
    for (int i = c.imin(); i <= c.imax(); i++) {
      for (int j = c.jmin() + bestSplitPos - 1; j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c2.push(i, j);
        }
      }
    }

    c1.recursiveSplit(clusteringMap, out, alignment, minPatchSize);
    c2.recursiveSplit(clusteringMap, out, alignment, minPatchSize);

    return true;
  }
  return false;
}

auto Cluster::splitCPatchHorizontally(const ClusteringMap &clusteringMap, vector<Cluster> &out,
                                      int alignment, int minPatchSize) const -> bool {
  double splitThresholdC = 0.3;

  const Cluster &c = (*this);
  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  int H = c.height();
  int W = c.width();

  int numOfEmptyBlocks = 0;
  int numOfNonEmptyBlocks = 0;

  for (int h = 0; h < H; h += alignment) {
    for (int w = 0; w < W; w += alignment) {
      bool isEmpty = true;

      for (int hh = h; hh < min(h + alignment, H); hh++) {
        int i = hh + c.imin();
        for (int ww = w; ww < min(w + alignment, W); ww++) {
          int j = ww + c.jmin();

          if (clusteringBuffer(i, j) == c.getClusterId()) {
            isEmpty = false;
            break;
          }
        } // ww
        if (!isEmpty) {
          break;
        }
      } // hh
      if (isEmpty) {
        numOfEmptyBlocks++;
      } else {
        numOfNonEmptyBlocks++;
      }
    } // w
  }   // h

  if (double(numOfNonEmptyBlocks) / (numOfEmptyBlocks + numOfNonEmptyBlocks) < splitThresholdC) {
    int bestSplitPos = roundToAlignment(H, alignment);

    Cluster c1(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());
    Cluster c2(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());

    for (int i = c.imin(); i < c.imin() + bestSplitPos + 1; i++) {
      for (int j = c.jmin(); j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c1.push(i, j);
        }
      }
    }
    for (int i = c.imin() + bestSplitPos - 1; i <= c.imax(); i++) {
      for (int j = c.jmin(); j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c2.push(i, j);
        }
      }
    }

    c1.recursiveSplit(clusteringMap, out, alignment, minPatchSize);
    c2.recursiveSplit(clusteringMap, out, alignment, minPatchSize);

    return true;
  }
  return false;
}

auto Cluster::splitLPatchVertically(const ClusteringMap &clusteringMap, vector<Cluster> &out,
                                    int alignment, int minPatchSize,
                                    const array<deque<int>, 2> &min_h_agg,
                                    const array<deque<int>, 2> &max_h_agg) const -> bool {
  double splitThresholdL = 0.9;

  const Cluster &c = (*this);
  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  int H = c.height();
  int W = c.width();

  int alignedImsize = roundToAlignment(W, alignment) * roundToAlignment(H, alignment);

  int minArea = alignedImsize;
  int bestSplitPos = 0;
  for (int w = minPatchSize; w < W - minPatchSize; w++) {
    int currArea = roundToAlignment(w + 1, alignment) *
                       roundToAlignment(max_h_agg[0][w] - min_h_agg[0][w], alignment) +
                   roundToAlignment(W - w - 1, alignment) *
                       roundToAlignment(max_h_agg[1][w] - min_h_agg[1][w], alignment);

    if (minArea > currArea) {
      minArea = currArea;
      bestSplitPos = w;
    }
  }

  if ((bestSplitPos != 0) && double(minArea) / alignedImsize < splitThresholdL) {
    Cluster c1(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());
    Cluster c2(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());

    for (int i = c.imin(); i <= c.imax(); i++) {
      for (int j = c.jmin(); j < c.jmin() + bestSplitPos + 1; j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c1.push(i, j);
        }
      }
    }
    for (int i = c.imin(); i <= c.imax(); i++) {
      for (int j = c.jmin() + bestSplitPos - 1; j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c2.push(i, j);
        }
      }
    }

    c1.recursiveSplit(clusteringMap, out, alignment, minPatchSize);
    c2.recursiveSplit(clusteringMap, out, alignment, minPatchSize);

    return true;
  }
  return false;
}

auto Cluster::recursiveSplit(const ClusteringMap &clusteringMap, vector<Cluster> &out,
                             int alignment, int minPatchSize) const -> vector<Cluster> {
  bool splitted = false;

  int maxNonsplittableSize = 64;

  const Cluster &c = (*this);
  const auto &clusteringBuffer = clusteringMap.getPlane(0);

  int H = c.height();
  int W = c.width();

  vector<int> min_w;
  vector<int> max_w;
  for (int h = 0; h < H; h++) {
    min_w.push_back(W - 1);
    max_w.push_back(0);
  }
  vector<int> min_h;
  vector<int> max_h;
  for (int w = 0; w < W; w++) {
    min_h.push_back(H - 1);
    max_h.push_back(0);
  }

  for (int h = 0; h < H; h++) {
    int i = h + c.imin();

    for (int w = 0; w < W; w++) {
      int j = w + c.jmin();
      if (clusteringBuffer(i, j) == c.getClusterId()) {
        min_w[h] = w;
        break;
      }
    }

    for (int w = W - 1; w >= 0; w--) {
      int j = w + c.jmin();
      if (clusteringBuffer(i, j) == c.getClusterId()) {
        max_w[h] = w;
        break;
      }
    }
  }

  for (int w = 0; w < W; w++) {
    int j = w + c.jmin();

    for (int h = 0; h < H; h++) {
      int i = h + c.imin();
      if (clusteringBuffer(i, j) == c.getClusterId()) {
        min_h[w] = h;
        break;
      }
    }

    for (int h = H - 1; h >= 0; h--) {
      int i = h + c.imin();
      if (clusteringBuffer(i, j) == c.getClusterId()) {
        max_h[w] = h;
        break;
      }
    }
  }

  auto min_w_agg = array<deque<int>, 2>{};
  auto max_w_agg = array<deque<int>, 2>{};
  auto min_h_agg = array<deque<int>, 2>{};
  auto max_h_agg = array<deque<int>, 2>{};

  min_w_agg[0].push_back(min_w[0]);
  max_w_agg[0].push_back(max_w[0]);
  for (int h = 1; h < H; h++) {
    min_w_agg[0].push_back(min(min_w_agg[0][h - 1], min_w[h]));
    max_w_agg[0].push_back(max(max_w_agg[0][h - 1], max_w[h]));
  }
  min_w_agg[1].push_front(min_w[H - 1]);
  max_w_agg[1].push_front(max_w[H - 1]);
  for (int h = H - 2; h >= 0; h--) {
    min_w_agg[1].push_front(min(min_w_agg[1][0], min_w[h]));
    max_w_agg[1].push_front(max(max_w_agg[1][0], max_w[h]));
  }

  min_h_agg[0].push_back(min_h[0]);
  max_h_agg[0].push_back(max_h[0]);
  for (int w = 1; w < W; w++) {
    min_h_agg[0].push_back(min(min_h_agg[0][w - 1], min_h[w]));
    max_h_agg[0].push_back(max(max_h_agg[0][w - 1], max_h[w]));
  }
  min_h_agg[1].push_front(min_h[W - 1]);
  max_h_agg[1].push_front(max_h[W - 1]);
  for (int w = W - 2; w >= 0; w--) {
    min_h_agg[1].push_front(min(min_h_agg[1][0], min_h[w]));
    max_h_agg[1].push_front(max(max_h_agg[1][0], max_h[w]));
  }

  if (W > H) { // split vertically
    if (W > maxNonsplittableSize) {
      splitted =
          splitLPatchVertically(clusteringMap, out, alignment, minPatchSize, min_h_agg, max_h_agg);
      if (!splitted) {
        splitted = splitCPatchVertically(clusteringMap, out, alignment, minPatchSize);
      }
    }
  } else { // split horizontally
    if (H > maxNonsplittableSize) {
      splitted = splitLPatchHorizontally(clusteringMap, out, alignment, minPatchSize, min_w_agg,
                                         max_w_agg);
      if (!splitted) {
        splitted = splitCPatchHorizontally(clusteringMap, out, alignment, minPatchSize);
      }
    }
  }

  if (!splitted) {
    out.push_back(c);
  }

  return out;
}

auto Cluster::split(const ClusteringMap &clusteringMap, int overlap) const
    -> pair<Cluster, Cluster> {
  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  const Cluster &c = *this;
  assert(!c.isBasicView());
  Cluster c1(c.getViewId(), false, c.getClusterId(), c.getEntityId());
  Cluster c2(c.getViewId(), false, c.getClusterId(), c.getEntityId());

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
  c1.numActivePixels_ = (c.numActivePixels_ * c1.filling_) / c.filling_; // Approximation
  c2.numActivePixels_ = c.numActivePixels_ - c1.numActivePixels_;        // Approximation
  return pair<Cluster, Cluster>(c1, c2);
}

auto Cluster::retrieve(int viewId, const Mask &maskMap, int firstClusterId, bool isBasicView,
                       bool enableMerging) -> pair<ClusterList, ClusteringMap> {
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
    Cluster cluster(viewId, isBasicView, clusterId, 0);
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

    int subClusterId = clusterId;

    if (enableMerging) {
      // Patch Merging
      int i_top = cluster.imin(), i_bottom = cluster.imax();
      int j_left = cluster.jmin(), j_right = cluster.jmax();

      auto subRegionGrowing = [&](int ID) {
        Cluster subCluster(viewId, isBasicView, subClusterId, cluster.getEntityId());
        while (!candidates.empty()) {
          const std::array<int, 2> &current = candidates.front();
          int a = current[0];
          int b = current[1];

          subCluster.push(a, b);
          clusteringBuffer(a, b) = static_cast<uint16_t>(ID);

          if (0 < a) {
            if (clusteringBuffer(a - 1, b) == ACTIVE) {
              candidates.push({a - 1, b});
              subCluster.push(a - 1, b);
              clusteringBuffer(a - 1, b) = static_cast<uint16_t>(ID);
            }
            if (0 < b) {
              if (clusteringBuffer(a - 1, b - 1) == ACTIVE) {
                candidates.push({a - 1, b - 1});
                subCluster.push(a - 1, b - 1);
                clusteringBuffer(a - 1, b - 1) = static_cast<uint16_t>(ID);
              }
            }
            if (b < B - 1) {
              if (clusteringBuffer(a - 1, b + 1) == ACTIVE) {
                candidates.push({a - 1, b + 1});
                subCluster.push(a - 1, b + 1);
                clusteringBuffer(a - 1, b + 1) = static_cast<uint16_t>(ID);
              }
            }
          }
          if (a < A - 1) {
            if (clusteringBuffer(a + 1, b) == ACTIVE) {
              candidates.push({a + 1, b});
              subCluster.push(a + 1, b);
              clusteringBuffer(a + 1, b) = static_cast<uint16_t>(ID);
            }
            if (0 < b) {
              if (clusteringBuffer(a + 1, b - 1) == ACTIVE) {
                candidates.push({a + 1, b - 1});
                subCluster.push(a + 1, b - 1);
                clusteringBuffer(a + 1, b - 1) = static_cast<uint16_t>(ID);
              }
            }
            if (b < B - 1) {
              if (clusteringBuffer(a + 1, b + 1) == ACTIVE) {
                candidates.push({a + 1, b + 1});
                subCluster.push(a + 1, b + 1);
                clusteringBuffer(a + 1, b + 1) = static_cast<uint16_t>(ID);
              }
            }
          }
          if (0 < b) {
            if (clusteringBuffer(a, b - 1) == ACTIVE) {
              candidates.push({a, b - 1});
              subCluster.push(a, b - 1);
              clusteringBuffer(a, b - 1) = static_cast<uint16_t>(ID);
            }
          }
          if (b < B - 1) {
            if (clusteringBuffer(a, b + 1) == ACTIVE) {
              candidates.push({a, b + 1});
              subCluster.push(a, b + 1);
              clusteringBuffer(a, b + 1) = static_cast<uint16_t>(ID);
            }
          }
          candidates.pop();
        }
        clusterList.push_back(subCluster);
        clustered += subCluster.getFilling();
      };

      // left side
      if (j_left != 0) {
        for (int i_unit = i_top; i_unit <= i_bottom; i_unit++) {
          if (clusteringBuffer(i_unit, j_left - 1) == ACTIVE) {
            subClusterId = subClusterId + 1;
            candidates.push({i_unit, j_left - 1});
            subRegionGrowing(subClusterId);
          }
        }
      }
      // right side
      if (j_right != B - 1) {
        for (int i_unit = i_top; i_unit <= i_bottom; i_unit++) {
          if (clusteringBuffer(i_unit, j_right + 1) == ACTIVE) {
            subClusterId = subClusterId + 1;
            candidates.push({i_unit, j_right + 1});
            subRegionGrowing(subClusterId);
          }
        }
      }
      // bottom side
      if (i_bottom != A - 1) {
        for (int j_unit = j_left; j_unit <= j_right; j_unit++) {
          if (clusteringBuffer(i_bottom + 1, j_unit) == ACTIVE) {
            subClusterId = subClusterId + 1;
            candidates.push({i_bottom + 1, j_unit});
            subRegionGrowing(subClusterId);
          }
        }
      }
      if (!isBasicView) {
        for (int i_inter = i_top; i_inter <= i_bottom; i_inter++) {
          for (int j_inter = j_left; j_inter <= j_right; j_inter++) {
            if (clusteringBuffer(i_inter, j_inter) == ACTIVE) {
              clusteringBuffer(i_inter, j_inter) = static_cast<uint16_t>(clusterId);
            }
          }
        }
      }
      // Patch Merging END
    }

    // Update seed & compute # Active Pixels In Patch
    auto prevIter = iter_seed;
    iter_seed = find_if(iter_seed + 1, activeList.end(),
                        [&clusteringBuffer](int i) { return (clusteringBuffer[i] == ACTIVE); });
    auto currentIter = iter_seed;
    auto counter = int(distance(prevIter, currentIter));
    cluster.numActivePixels_ = counter;

    // Updating output
    if (isBasicView) {
      if (!clusterList.empty()) {
        clusterList[0] = Cluster::merge(clusterList[0], cluster);
      } else {
        clusterList.push_back(cluster);
      }

      clustered = clusterList[0].getFilling();
    } else {
      clustered += cluster.getFilling();

      clusterList.push_back(cluster);
      clusterId = subClusterId + 1; // Patch Merging
    }
  }

  return out;
}

} // namespace TMIV::Packer