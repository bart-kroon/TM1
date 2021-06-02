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

#include "../src/IncrementalSynthesizer.h"
#include "../src/PrunedMesh.h"
#include <TMIV/Common/Thread.h>
#include <TMIV/Encoder/Encoder.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/Renderer/Rasterizer.h>
#include <TMIV/Renderer/reprojectPoints.h>

namespace TMIV::Encoder {
auto findCentralBasicView(MivBitstream::ViewParamsList paramsList) -> size_t {
  Common::Vec3f minPos{paramsList[0].pose.position};
  Common::Vec3f maxPos{paramsList[0].pose.position};

  for (size_t i = 1; i < paramsList.size(); ++i) {
    minPos.x() = std::min(minPos.x(), paramsList[i].pose.position.x());
    minPos.y() = std::min(minPos.y(), paramsList[i].pose.position.y());
    minPos.z() = std::min(minPos.z(), paramsList[i].pose.position.z());
    maxPos.x() = std::max(maxPos.x(), paramsList[i].pose.position.x());
    maxPos.y() = std::max(maxPos.y(), paramsList[i].pose.position.y());
    maxPos.z() = std::max(maxPos.z(), paramsList[i].pose.position.z());
  }

  const Common::Vec3f avgPos = (maxPos + minPos) / 2;

  float bestDist = std::numeric_limits<float>::max();
  size_t refView = 0;

  for (size_t i = 1; i < paramsList.size(); ++i) {
    if (paramsList[i].isBasicView) {
      const Common::Vec3f pos{paramsList[i].pose.position};
      const Common::Vec3f sqrDist{(pos.x() - avgPos.x()) * (pos.x() - avgPos.x()),
                                  (pos.y() - avgPos.y()) * (pos.y() - avgPos.y()),
                                  (pos.z() - avgPos.z()) * (pos.z() - avgPos.z())};

      const auto dist = sqrtf(sqrDist.x() + sqrDist.y() + sqrDist.z());

      if (dist < bestDist) {
        bestDist = dist;
        refView = i;
      }
    }
  }

  return refView;
}

auto assessColorConsistency(Common::MVD16Frame views, MivBitstream::ViewParamsList params)
    -> std::vector<Common::Mat<Common::Vec3i>> {
  std::vector<Common::Mat<Common::Vec3i>> colorCorrectionMaps1Frame;

  float m_maxDepthError = 0.1F;
  float m_maxLumaError = 0.04F;
  const Renderer::AccumulatingPixel<Common::Vec3f> tmpConfig{10.0, -100.0, 3.0, 5.0};
  const auto refViewId = findCentralBasicView(params);

  std::vector<std::unique_ptr<TMIV::Pruner::IncrementalSynthesizer>> synthesizers;
  synthesizers.clear();

  for (size_t i = 0; i < params.size(); ++i) {
    const auto depthTransform = MivBitstream::DepthTransform{params[i].dq, 16};
    synthesizers.emplace_back(std::make_unique<TMIV::Pruner::IncrementalSynthesizer>(
        tmpConfig, params[i].ci.projectionPlaneSize(), i,
        depthTransform.expandDepth(views[i].depth), expandLuma(views[i].texture),
        expandTexture(yuv444p(views[i].texture))));
  }

  std::vector<Common::Frame<Common::YUV400P8>> masks;
  masks.clear();
  masks.reserve(views.size());
  std::transform(
      std::cbegin(params), std::cend(params), std::cbegin(views), back_inserter(masks),
      [](const MivBitstream::ViewParams &viewParams, const Common::TextureDepth16Frame &view) {
        auto mask =
            Common::Frame<Common::YUV400P8>{viewParams.ci.ci_projection_plane_width_minus1() + 1,
                                            viewParams.ci.ci_projection_plane_height_minus1() + 1};

        std::transform(std::cbegin(view.depth.getPlane(0)), std::cend(view.depth.getPlane(0)),
                       std::begin(mask.getPlane(0)),
                       [ot = MivBitstream::OccupancyTransform{viewParams}](auto x) {
                         // #94: When there are invalid pixels in a basic view, these
                         // should be excluded from the pruning mask
                         return ot.occupant(x) ? uint8_t{255} : uint8_t{};
                       });
        return mask;
      });

  auto refView = views[refViewId];

  auto [ivertices, triangles, attributes] =
      Pruner::unprojectPrunedView(refView, params[refViewId], masks[refViewId].getPlane(0));

  int W = refView.texture.getWidth();
  int H = refView.texture.getHeight();

  for (auto &s : synthesizers) {
    Common::Mat<Common::Vec3i> currentCCMap;
    currentCCMap.resize(H, W);

    if (s->index != refViewId) {
      auto overtices = Pruner::project(ivertices, params[refViewId], params[s->index]);
      Pruner::weightedSphere(params[s->index].ci, overtices, triangles);
      s->rasterizer.submit(overtices, attributes, triangles);
      s->rasterizer.run();

      auto j = std::begin(s->reference);
      auto jY = std::begin(s->referenceY);
      auto jYUV = std::begin(s->referenceYUV);

      int pp = 0;

      s->rasterizer.visit([&](const Renderer::PixelValue<Common::Vec3f> &x) {
        if (x.normDisp > 0) {
          const auto depthError = x.depth() / *j - 1.F;
          const auto lumaError = std::get<0>(x.attributes()).x() - *(jY);

          const auto h = pp / W;
          const auto w = pp % W;

          if (h >= H) {
            return false;
          }

          if (fabs(depthError) < m_maxDepthError && fabs(lumaError) < m_maxLumaError) {
            currentCCMap(h, w).x() = static_cast<int32_t>(lumaError * 1023);
            auto chromaError = std::get<0>(x.attributes()).y() - jYUV->y();
            currentCCMap(h, w).y() = static_cast<int32_t>(chromaError * 1023);
            chromaError = std::get<0>(x.attributes()).z() - jYUV->z();
            currentCCMap(h, w).z() = static_cast<int32_t>(chromaError * 1023);
          } else {
            currentCCMap(h, w) = {};
          }
        }

        ++j;
        ++jY;
        ++jYUV;
        ++pp;

        return true;
      });
    }

    colorCorrectionMaps1Frame.push_back(std::move(currentCCMap));
  }
  return colorCorrectionMaps1Frame;
}
} // namespace TMIV::Encoder
