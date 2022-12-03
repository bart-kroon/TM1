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

#ifndef TMIV_VIEWOPTIMIZER_BASICVIEWALLOCATOR_H
#define TMIV_VIEWOPTIMIZER_BASICVIEWALLOCATOR_H

#include "AbstractViewSelector.h"
#include "KMedoidsCost.h"

namespace TMIV::ViewOptimizer {
class BasicViewAllocator : public AbstractViewSelector {
public:
  using Centroids = KMedoidsCost::Centroids;
  using Positions = std::vector<Common::Vec3d>;

  BasicViewAllocator(const Common::Json &rootNode, const Common::Json &componentNode);

protected:
  [[nodiscard]] auto isBasicView(double weight) const -> std::vector<bool> override;

private:
  // Calculate 'k'
  [[nodiscard]] auto basicViewCount() const -> size_t;
  [[nodiscard]] auto lumaSamplesPerSourceViewSortedDesc() const -> std::vector<size_t>;

  // Prepare cost calculation
  [[nodiscard]] auto viewPositions() const -> Positions;
  [[nodiscard]] auto forwardView(const Positions &pos) const -> size_t;
  static auto sqDistanceMatrix(const Positions &pos, double weight) -> Common::Mat<double>;

  // Partitioning around medians [https://en.wikipedia.org/wiki/K-medoids#Algorithms]
  static auto selectInitialCentroids(const KMedoidsCost &cost, size_t first, size_t k) -> Centroids;
  static auto updateCentroids(const KMedoidsCost &cost, Centroids centroids)
      -> std::optional<Centroids>;

  int32_t m_numGroups{};
  int32_t m_maxLumaPictureSize{};
  int32_t m_maxAtlases{};
  int32_t m_minNonCodedViews{}; // for evaluation purposes
  double m_maxBasicViewFraction{};
};
} // namespace TMIV::ViewOptimizer

#endif
