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

#ifndef TMIV_ATLASCONSTRUCTOR_CLUSTER_H
#define TMIV_ATLASCONSTRUCTOR_CLUSTER_H

#include <TMIV/Common/Frame.h>

#include <queue>
#include <tuple>

namespace TMIV::Packer {
using ClusteringMap = Common::Frame<>;
using ClusteringMapList = std::vector<ClusteringMap>;

class Cluster;
using ClusterList = std::vector<Cluster>;

class Cluster {
public:
  Cluster() = default;
  Cluster(int32_t viewIdx, bool isBasicView, bool isSemiBasicView, int32_t clusterId,
          int32_t entityId);
  Cluster(const Cluster &) = default;
  Cluster(Cluster &&) = default;
  auto operator=(const Cluster &) -> Cluster & = default;
  auto operator=(Cluster &&) -> Cluster & = default;
  ~Cluster() = default;

  void push(int32_t i, int32_t j);
  [[nodiscard]] auto getViewIdx() const -> int32_t { return viewIdx_; }
  [[nodiscard]] auto getClusterId() const -> int32_t { return clusterId_; }
  [[nodiscard]] auto getEntityId() const -> int32_t { return entityId_; }
  [[nodiscard]] auto getNumActivePixels() const -> int32_t { return numActivePixels_; }
  [[nodiscard]] auto imin() const -> int32_t { return imin_; }
  [[nodiscard]] auto jmin() const -> int32_t { return jmin_; }
  [[nodiscard]] auto imax() const -> int32_t { return imax_; }
  [[nodiscard]] auto jmax() const -> int32_t { return jmax_; }
  [[nodiscard]] auto width() const -> int32_t { return (jmax_ - jmin_ + 1); }
  [[nodiscard]] auto height() const -> int32_t { return (imax_ - imin_ + 1); }
  [[nodiscard]] auto getArea() const -> int32_t { return width() * height(); }
  [[nodiscard]] auto getInformationDensity() const -> int32_t { return information_density_; }
  [[nodiscard]] auto getPriority() const -> int32_t { return information_priority_; }

  [[nodiscard]] auto getMinSize() const -> int32_t { return std::min(width(), height()); }
  [[nodiscard]] auto split(const ClusteringMap &clusteringMap, int32_t overlap) const
      -> std::pair<Cluster, Cluster>;
  [[nodiscard]] constexpr auto isBasicView() const noexcept { return m_isBasicView; }
  [[nodiscard]] constexpr auto isSemiBasicView() const noexcept { return m_isSemiBasicView; }
  auto numActivePixels() -> int32_t & { return numActivePixels_; }
  void setPriority(int32_t priority) { information_priority_ = priority; }
  void calculateInformationDensity(const ClusteringMap &clusteringMap,
                                   const Common::Frame<uint32_t> &informationMap);

  void recursiveSplit(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                      int32_t alignment, int32_t minPatchSize) const;

  void recursiveInformationSplit(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                                 int32_t alignment, int32_t minPatchSize,
                                 const Common::FrameList<uint32_t> &information,
                                 uint32_t nonSplitInformation) const;

  static auto Empty() -> Cluster {
    Cluster out;
    out.imin_ = 0;
    out.imax_ = 0;
    out.jmin_ = 0;
    out.jmax_ = 0;
    return out;
  }
  static auto setEntityId(const Cluster &c, int32_t entityId) -> Cluster;
  static auto align(const Cluster &c, int32_t alignment) -> Cluster;
  static auto merge(const Cluster &c1, const Cluster &c2) -> Cluster;

private:
  auto splitLPatchVertically(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                             int32_t alignment, int32_t minPatchSize,
                             const std::array<std::deque<int32_t>, 2> &min_h_agg,
                             const std::array<std::deque<int32_t>, 2> &max_h_agg) const -> bool;
  auto splitLPatchHorizontally(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                               int32_t alignment, int32_t minPatchSize,
                               const std::array<std::deque<int32_t>, 2> &min_w_agg,
                               const std::array<std::deque<int32_t>, 2> &max_w_agg) const -> bool;
  auto splitCPatchVertically(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                             int32_t alignment, int32_t minPatchSize) const -> bool;
  auto splitCPatchHorizontally(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                               int32_t alignment, int32_t minPatchSize) const -> bool;
  auto splitnUnevenInformationPatchVertically(const ClusteringMap &clusteringMap,
                                              std::vector<Cluster> &out, int32_t alignment,
                                              int32_t minPatchSize,
                                              const Common::FrameList<uint32_t> &information,
                                              uint32_t nonSplitInformation) const -> bool;
  auto splitUnevenInformationPatchHorizontally(const ClusteringMap &clusteringMap,
                                               std::vector<Cluster> &out, int32_t alignment,
                                               int32_t minPatchSize,
                                               const Common::FrameList<uint32_t> &information,
                                               uint32_t nonSplitInformation) const -> bool;
  [[nodiscard]] auto createAggregatedQueues(const ClusteringMap &clusteringMap,
                                            bool aggregateHorizontally) const
      -> std::tuple<std::array<std::deque<int32_t>, 2>, std::array<std::deque<int32_t>, 2>>;
  [[nodiscard]] auto computeMinAndMaxVectors(const ClusteringMap &clusteringMap,
                                             bool aggregateHorizontally) const
      -> std::tuple<std::vector<int32_t>, std::vector<int32_t>>;

  int32_t viewIdx_ = 0;
  bool m_isBasicView{};
  bool m_isSemiBasicView{};
  int32_t clusterId_ = 0;
  int32_t entityId_ = 0;
  int32_t numActivePixels_ = 0;
  int32_t imin_ = std::numeric_limits<int32_t>::max();
  int32_t jmin_ = std::numeric_limits<int32_t>::max();
  int32_t imax_ = std::numeric_limits<int32_t>::min();
  int32_t jmax_ = std::numeric_limits<int32_t>::min();
  int32_t information_density_ = 0;
  int32_t information_priority_ = 0;
};
} // namespace TMIV::Packer

#endif
