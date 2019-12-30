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

#ifndef _TMIV_ATLASCONSTRUCTOR_CLUSTER_H_
#define _TMIV_ATLASCONSTRUCTOR_CLUSTER_H_

#include <TMIV/AtlasConstructor/IPruner.h>

namespace TMIV::AtlasConstructor {

using ClusteringMap = Common::Frame<Common::YUV400P16>;
using ClusteringMapList = std::vector<ClusteringMap>;

class Cluster;
using ClusterList = std::vector<Cluster>;

class Cluster {
protected:
  int viewId_ = 0;
  int clusterId_ = 0;
  int entityId_ = 0;
  int numActivePixels_ = 0;
  int imin_ = std::numeric_limits<int>::max();
  int jmin_ = std::numeric_limits<int>::max();
  int imax_ = std::numeric_limits<int>::min();
  int jmax_ = std::numeric_limits<int>::min();
  int filling_ = 0;

public:
  Cluster() = default;
  Cluster(int viewId, int clusterId);
  Cluster(int viewId, int clusterId, int entityId);
  Cluster(const Cluster &) = default;
  Cluster(Cluster &&) = default;
  Cluster &operator=(const Cluster &) = default;
  Cluster &operator=(Cluster &&) = default;
  ~Cluster() = default;

  void push(int i, int j);
  int getViewId() const { return viewId_; }
  int getClusterId() const { return clusterId_; }
  int getEntityId() const { return entityId_; }
  int getNumActivePixels() const { return numActivePixels_; }
  int imin() const { return imin_; }
  int jmin() const { return jmin_; }
  int imax() const { return imax_; }
  int jmax() const { return jmax_; }
  int getFilling() const { return filling_; }
  int width() const { return (jmax_ - jmin_ + 1); }
  int height() const { return (imax_ - imin_ + 1); }
  int getArea() const { return width() * height(); }
  int getMinSize() const { return std::min(width(), height()); }
  std::pair<Cluster, Cluster> split(const ClusteringMap &clusteringMap, int overlap) const;
  static Cluster Empty() {
    Cluster out;
    out.imin_ = 0;
    out.imax_ = 0;
    out.jmin_ = 0;
    out.jmax_ = 0;
    return out;
  }
  static Cluster setEntityId(Cluster &c, int entityId); 
  static Cluster align(const Cluster &c, int alignment);
  static Cluster merge(const Cluster &c1, const Cluster &c2);
  static std::pair<ClusterList, ClusteringMap> retrieve(int viewId, const Common::Mask &maskMap,
                                                        int firstClusterId = 0,
                                                        bool shouldNotBeSplit = false);
};

} // namespace TMIV::AtlasConstructor

#endif
