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

#include "LumaStdDev.h"
#include "PrunedMesh.h"

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>

namespace TMIV::Pruner {
namespace {
auto calculateCenterPosition(const MivBitstream::ViewParamsList &viewParamsList) -> Common::Vec3f {
  Common::Vec3f minPos{viewParamsList[0].pose.position};
  Common::Vec3f maxPos{viewParamsList[0].pose.position};

  for (size_t i = 1; i < viewParamsList.size(); ++i) {
    const Common::Vec3f currentPos{viewParamsList[i].pose.position};

    for (int32_t j = 0; j < 3; j++) {
      minPos[j] = std::min(minPos[j], currentPos[j]);
      maxPos[j] = std::max(maxPos[j], currentPos[j]);
    }
  }

  return (maxPos + minPos) / 2.F;
}

auto getDistanceToPosition(const MivBitstream::ViewParams &viewParams,
                           const Common::Vec3f &position) -> float {
  const auto differenceVector = viewParams.pose.position - position;
  return std::sqrt(differenceVector.x() * differenceVector.x() +
                   differenceVector.y() * differenceVector.y() +
                   differenceVector.z() * differenceVector.z());
}

auto findCentralBasicView(const MivBitstream::ViewParamsList &viewParamsList) -> size_t {
  PRECONDITION(!viewParamsList.empty());

  if (viewParamsList.size() == 1) {
    return 0;
  }

  const auto centerPos = calculateCenterPosition(viewParamsList);

  const auto viewClosestToCenter =
      std::min_element(std::next(std::cbegin(viewParamsList)), std::cend(viewParamsList),
                       [&centerPos](const auto &viewParameter1, const auto &viewParameter2) {
                         if (viewParameter1.isBasicView) {
                           const auto distance1 = getDistanceToPosition(viewParameter1, centerPos);
                           const auto distance2 = getDistanceToPosition(viewParameter2, centerPos);
                           return distance1 < distance2;
                         }
                         return false;
                       });

  return static_cast<size_t>(
      std::abs(std::distance(std::cbegin(viewParamsList), viewClosestToCenter)));
}

auto initSynthesizersForFrameAnalysis(const Common::DeepFrameList &views,
                                      const MivBitstream::ViewParamsList &viewParamsList,
                                      Renderer::AccumulatingPixel config)
    -> std::vector<std::unique_ptr<IncrementalSynthesizer>> {
  std::vector<std::unique_ptr<IncrementalSynthesizer>> synthesizers{};
  for (size_t i = 0; i < viewParamsList.size(); ++i) {
    const auto geoBitDepth = views[i].geometry.getBitDepth();
    const auto depthTransform = MivBitstream::DepthTransform{viewParamsList[i].dq, geoBitDepth};

    synthesizers.emplace_back(std::make_unique<IncrementalSynthesizer>(
        config, viewParamsList[i].ci.projectionPlaneSize(), i,
        depthTransform.expandDepth(views[i].geometry), expandLuma(views[i].texture),
        expandTexture(yuv444(views[i].texture))));
  }
  return synthesizers;
}

auto initMasksForFrameAnalysis(const Common::DeepFrameList &views,
                               const MivBitstream::ViewParamsList &viewParamsList)
    -> Common::FrameList<uint8_t> {
  Common::FrameList<uint8_t> masks{};
  std::transform(std::cbegin(viewParamsList), std::cend(viewParamsList), std::cbegin(views),
                 back_inserter(masks),
                 [](const MivBitstream::ViewParams &viewParams, const Common::DeepFrame &view) {
                   auto mask = Common::Frame<uint8_t>::lumaOnly(
                       {viewParams.ci.ci_projection_plane_width_minus1() + 1,
                        viewParams.ci.ci_projection_plane_height_minus1() + 1});

                   std::transform(std::cbegin(view.geometry.getPlane(0)),
                                  std::cend(view.geometry.getPlane(0)),
                                  std::begin(mask.getPlane(0)),
                                  [ot = MivBitstream::OccupancyTransform{viewParams}](auto x) {
                                    // #94: When there are invalid pixels in a basic view, these
                                    // should be excluded from the pruning mask
                                    return ot.occupant(x) ? uint8_t{255} : uint8_t{};
                                  });
                   return mask;
                 });
  return masks;
}

auto isAnyNeighboringPixelSimilar(const int32_t H, const int32_t W, int32_t pixelIdx,
                                  const int32_t numOfBins2, float middleVal,
                                  TMIV::Common::Mat<float>::const_iterator &jY) -> bool {
  const auto h = pixelIdx / W;
  const auto w = pixelIdx % W;

  for (int32_t hh = -1; hh <= 1; hh++) {
    for (int32_t ww = -1; ww <= 1; ww++) {
      if (h + hh < 0 || h + hh >= H || w + ww < 0 || w + ww >= W) {
        continue;
      }
      const auto offset = hh * W + ww;
      const float lumaError = std::abs(middleVal - *(jY + offset)) * static_cast<float>(numOfBins2);
      if (lumaError < 0.5F) {
        return true;
      }
    }
  }

  return false;
}

auto calculateStdDev(const std::vector<int32_t> &differenceHistogram) -> std::optional<float> {
  const int64_t numSamples =
      std::accumulate(std::cbegin(differenceHistogram), std::cend(differenceHistogram), 0);
  if (numSamples == 0) {
    return std::nullopt;
  }

  const auto numOfBins2 = static_cast<int32_t>(differenceHistogram.size()) / 2;
  int64_t sum = 0;
  for (int32_t bin = 0; bin < static_cast<int32_t>(differenceHistogram.size()); ++bin) {
    sum += (bin - numOfBins2) * differenceHistogram[bin];
  }

  const float average = static_cast<float>(sum) / static_cast<float>(numSamples);
  sum = 0;

  for (int32_t bin = 0; bin < static_cast<int32_t>(differenceHistogram.size()); ++bin) {
    const float value = average - static_cast<float>(bin) + static_cast<float>(numOfBins2);
    sum += static_cast<int64_t>(static_cast<float>(differenceHistogram[bin]) * value * value);
  }

  return std::sqrt(static_cast<float>(sum) / static_cast<float>(numSamples)) / 4.0F;
}
} // namespace

auto calculateLumaStdDev(const Common::DeepFrameList &views,
                         const MivBitstream::ViewParamsList &viewParamsList,
                         const Renderer::AccumulatingPixel &config, float maxDepthError)
    -> std::optional<float> {
  const int32_t numBins = 512;
  std::vector<int32_t> differenceHistogram(numBins, 0);
  const int32_t numBins2 = numBins / 2U;

  const auto synthesizers = initSynthesizersForFrameAnalysis(views, viewParamsList, config);

  const size_t refViewId = findCentralBasicView(viewParamsList);
  const auto &refView = views[refViewId];

  const auto masks = initMasksForFrameAnalysis(views, viewParamsList);
  auto [ivertices, triangles, attributes] =
      unprojectPrunedView(refView, viewParamsList[refViewId], masks[refViewId].getPlane(0));

  // compare reprojected points
  for (const auto &s : synthesizers) {
    if (s->index == refViewId) {
      continue;
    }

    auto overtices = project(ivertices, viewParamsList[refViewId], viewParamsList[s->index]);
    weightedSphere(viewParamsList[s->index].ci, overtices, triangles);
    s->rasterizer.submit(overtices, attributes, triangles);
    s->rasterizer.run();

    const auto W = static_cast<int32_t>(s->reference.width());
    const auto H = static_cast<int32_t>(s->reference.height());

    auto j = std::begin(s->reference);
    auto jY = std::begin(s->referenceY);
    auto jYUV = std::begin(s->referenceYUV);

    int32_t pixelIdx = 0;

    s->rasterizer.visit([&](const Renderer::PixelValue &x) {
      if (x.normDisp > 0) {
        const auto depthError = x.depth() / *j - 1.F;

        const auto relativeErrorThreshold = 0.1F;
        if (std::abs(depthError) < (maxDepthError * relativeErrorThreshold)) {
          const auto middleVal = x.color.x();
          if (isAnyNeighboringPixelSimilar(H, W, pixelIdx, numBins2, middleVal, jY)) {
            const auto lumaError = x.color.x() - *(jY);
            const auto binIdx = std::clamp<size_t>(
                static_cast<size_t>(numBins2) +
                    static_cast<size_t>(lroundf(lumaError * static_cast<float>(numBins2))),
                0U, differenceHistogram.size());
            differenceHistogram[binIdx]++;
          }
        }
      }

      j++;
      jY++;
      jYUV++;
      pixelIdx++;
      return true;
    });
  }

  return calculateStdDev(differenceHistogram);
}
} // namespace TMIV::Pruner
