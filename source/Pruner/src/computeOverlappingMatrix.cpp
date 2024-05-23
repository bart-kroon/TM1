/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#include "computeOverlappingMatrix.h"

namespace TMIV::Pruner {
namespace {
using PointCloud = std::vector<Common::Vec3f>;
using PointCloudList = std::vector<PointCloud>;

auto getPointCloud(const Renderer::ProjectionHelper &helper, uint32_t N) -> PointCloud {
  PointCloud pointCloud;

  float step = 1.F / static_cast<float>(N - 1U);
  auto depthRange = Common::Vec2f{1.F / helper.getViewParams().dq.dq_norm_disp_high(),
                                  1.F / helper.getViewParams().dq.dq_norm_disp_low()};

  float x = 0.F;

  for (uint32_t i = 0U; i < N; i++) {
    float y = 0.F;

    float px = x * helper.getViewParams().ci.projectionPlaneSizeF().x();

    for (uint32_t j = 0U; j < N; j++) {
      float d = depthRange.x();

      float py = y * helper.getViewParams().ci.projectionPlaneSizeF().y();

      for (uint32_t k = 0U; k < N; k++) {
        pointCloud.emplace_back(helper.doUnprojection({px, py}, d));

        d += step * (depthRange.y() - depthRange.x());
      }

      y += step;
    }

    x += step;
  }

  return pointCloud;
}

auto getPointCloudList(const Renderer::ProjectionHelperList &sourceHelperList, uint32_t N)
    -> PointCloudList {
  PointCloudList pointCloudList;

  for (const auto &helper : sourceHelperList) {
    pointCloudList.emplace_back(getPointCloud(helper, N));
  }

  return pointCloudList;
}

auto getOverlapping(const Renderer::ProjectionHelperList &sourceHelperList,
                    const PointCloudList &pointCloudList, size_t firstId, size_t secondId)
    -> float {
  size_t N = 0;

  const Renderer::ProjectionHelper &secondHelper = sourceHelperList[secondId];
  const PointCloud &firstPointCloud = pointCloudList[firstId];

  for (const auto &P : firstPointCloud) {
    auto p = secondHelper.doProjection(P);

    if (Renderer::isValidDepth(p.second) && secondHelper.isInsideViewport(p.first)) {
      N++;
    }
  }

  return static_cast<float>(N) / static_cast<float>(firstPointCloud.size());
}
} // namespace

auto computeOverlappingMatrix(const Renderer::ProjectionHelperList &sourceHelperList)
    -> Common::Mat<float> {
  auto pointCloudList = getPointCloudList(sourceHelperList, 16);
  size_t K = sourceHelperList.size();
  Common::Mat<float> overlappingMatrix({K, K});

  for (size_t i = 0; i < K; i++) {
    for (size_t j = 0; j < K; j++) {
      if (i != j) {
        overlappingMatrix(i, j) = getOverlapping(sourceHelperList, pointCloudList, i, j);
      } else {
        overlappingMatrix(i, j) = 1.F;
      }
    }
  }

  return overlappingMatrix;
}
} // namespace TMIV::Pruner
