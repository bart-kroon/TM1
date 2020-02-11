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

#include <TMIV/Common/Thread.h>
#include <TMIV/DepthQualityAssessor/DepthQualityAssessor.h>
#include <TMIV/Metadata/DepthOccupancyTransform.h>
#include <TMIV/Renderer/reprojectPoints.h>

using namespace TMIV::Common;
using namespace TMIV::Renderer;
using namespace TMIV::Metadata;

namespace TMIV::DepthQualityAssessor {

namespace {
template <typename MAT>
auto textureGather(const MAT &m, const Vec2f &p) -> stack::Vec4<typename MAT::value_type> {
  stack::Vec4<typename MAT::value_type> fetchedValues;

  int w_last = static_cast<int>(m.width()) - 1;
  int h_last = static_cast<int>(m.height()) - 1;

  int x0 = clamp(ifloor(p.x() - 0.5F), 0, w_last);
  int y0 = clamp(ifloor(p.y() - 0.5F), 0, h_last);

  int x1 = std::min(x0 + 1, w_last);
  int y1 = std::min(y0 + 1, h_last);

  fetchedValues[0] = m(y1, x0);
  fetchedValues[1] = m(y1, x1);
  fetchedValues[2] = m(y0, x1);
  fetchedValues[3] = m(y0, x0);

  return fetchedValues;
}

template <typename SourceProjectionType>
auto isLowDepthQuality(const Metadata::IvSequenceParams &ivSequenceParams,
                       const Common::MVD16Frame &sourceViews, float blendingFactor) -> bool {
  typename ProjectionHelper<SourceProjectionType>::List sourceHelperList{
      ivSequenceParams.viewParamsList};

  // Expand depth
  std::vector<Mat<float>> sourceDepthExpandedList;
  std::vector<Mat<Vec3f>> sourceUnprojectionList;

  sourceDepthExpandedList.reserve(sourceHelperList.size());
  sourceUnprojectionList.reserve(sourceHelperList.size());

  for (std::size_t viewId = 0; viewId < sourceHelperList.size(); viewId++) {

    const auto &sourceHelper = sourceHelperList[viewId];
    const auto occupancyTransform = OccupancyTransform{sourceHelper.getViewParams()};

    auto sourceDepthExpanded =
        DepthTransform<16>{sourceHelper.getViewParams()}.expandDepth(sourceViews[viewId].second);

    std::transform(sourceViews[viewId].second.getPlane(0).begin(),
                   sourceViews[viewId].second.getPlane(0).end(), sourceDepthExpanded.begin(),
                   sourceDepthExpanded.begin(), [&](std::uint16_t normDisp, float depthValue) {
                     return occupancyTransform.occupant(normDisp) ? depthValue : Common::NaN;
                   });

    Mat<Vec3f> sourceUnprojection({sourceDepthExpanded.height(), sourceDepthExpanded.width()});

    parallel_for(sourceDepthExpanded.width(), sourceDepthExpanded.height(),
                 [&](std::size_t y, std::size_t x) {
                   float z = sourceDepthExpanded(y, x);
                   if (sourceHelper.isValidDepth(z)) {
                     sourceUnprojection(y, x) = sourceHelper.doUnprojection(
                         Vec2f({static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F}), z);
                   } else {
                     sourceUnprojection(y, x) = Vec3f{Common::NaN, Common::NaN, Common::NaN};
                   }
                 });

    sourceDepthExpandedList.emplace_back(std::move(sourceDepthExpanded));
    sourceUnprojectionList.emplace_back(std::move(sourceUnprojection));
  }

  // Repojection for outlier detection
  std::atomic<bool> lowDepthQualityFlag = false;

  for (std::size_t firstId = 0; firstId < sourceHelperList.size(); firstId++) {

    const auto &firstHelper = sourceHelperList[firstId];
    const auto &firstDepth = sourceDepthExpandedList[firstId];

    for (std::size_t secondId = 0; secondId < sourceHelperList.size(); secondId++) {

      if (firstId != secondId) {

        const auto &secondUnprojection = sourceUnprojectionList[secondId];

        parallel_for(secondUnprojection.width(), secondUnprojection.height(),
                     [&](std::size_t y, std::size_t x) {
                       const auto &P = secondUnprojection(y, x);

                       if (!std::isnan(P.x())) {
                         auto p = firstHelper.doProjection(P);

                         if (isValidDepth(p.second) && firstHelper.isInsideViewport(p.first)) {

                           auto zOnFirst = textureGather(firstDepth, p.first);

                           if (std::all_of(zOnFirst.begin(), zOnFirst.end(), [&](float z) {
                                 return (!std::isnan(z) && (p.second < z * (1.F - blendingFactor)));
                               })) {

                             lowDepthQualityFlag = true;
                           }
                         }
                       }
                     });

        if (lowDepthQualityFlag) {
          break;
        }
      }
    }
  }

  return lowDepthQualityFlag;
}
} // namespace

DepthQualityAssessor::DepthQualityAssessor(const Common::Json & /*unused*/,
                                           const Common::Json &componentNode) {
  m_blendingFactor = componentNode.require("blendingFactor").asFloat();
}

auto DepthQualityAssessor::isLowDepthQuality(const Metadata::IvSequenceParams &ivSequenceParams,
                                             const Common::MVD16Frame &sourceViews) -> bool {
  return std::visit(
      [&](const auto &sourceProjection) -> bool {
        using SourceProjectionType = std::decay_t<decltype(sourceProjection)>;

        return TMIV::DepthQualityAssessor::isLowDepthQuality<SourceProjectionType>(
            ivSequenceParams, sourceViews, m_blendingFactor);
      },
      ivSequenceParams.viewParamsList[0].projection);
}
} // namespace TMIV::DepthQualityAssessor
