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

#include <TMIV/Renderer/AdditiveSynthesizer.h>

#include <TMIV/Common/LinAlg.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/Renderer/Engine.h>
#include <TMIV/Renderer/Rasterizer.h>
#include <TMIV/Renderer/reprojectPoints.h>

#include <cassert>
#include <cmath>
#include <future>
#include <numeric>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Renderer {
class AdditiveSynthesizer::Impl {
public:
  Impl(float rayAngleParam, float depthParam, float stretchingParam, float maxStretching)
      : m_rayAngleParam{rayAngleParam}, m_depthParam{depthParam},
        m_stretchingParam{stretchingParam}, m_maxStretching{maxStretching} {}

  Impl(const Impl &) = delete;
  Impl(Impl &&) = delete;
  auto operator=(const Impl &) -> Impl & = delete;
  auto operator=(Impl &&) -> Impl & = delete;
  ~Impl() = default;

  static auto affineTransformList(const ViewParamsList &viewParamsList,
                                  const CameraExtrinsics &target) {
    vector<AffineTransform> result;
    result.reserve(viewParamsList.size());
    for (const auto &source : viewParamsList) {
      result.emplace_back(source.ce, target);
    }
    return result;
  }

  static auto atlasVertices(const AtlasAccessUnit &atlas, const ViewParams &viewportParams) {
    SceneVertexDescriptorList result;
    const auto rows = atlas.frameSize().y();
    const auto cols = atlas.frameSize().x();
    result.reserve(rows * cols);

    const auto transformList = affineTransformList(atlas.viewParamsList, viewportParams.ce);

    vector<DepthTransform<10>> depthTransform;
    depthTransform.reserve(atlas.patchParamsList.size());
    for (const auto &patch : atlas.patchParamsList) {
      depthTransform.emplace_back(atlas.viewParamsList[patch.pduViewId()].dq, patch);
    }

    // For each used pixel in the atlas...
    for (int i_atlas = 0; i_atlas < rows; ++i_atlas) {
      for (int j_atlas = 0; j_atlas < cols; ++j_atlas) {
        const auto patchId = atlas.patchId(i_atlas, j_atlas);

        // Push dummy vertices to keep indexing simple
        if (patchId == unusedPatchId) {
          result.emplace_back();
          continue;
        }

        // Look up metadata
        assert(patchId < atlas.patchParamsList.size());
        const auto &patch = atlas.patchParamsList[patchId];
        assert(patch.pduViewId() < atlas.viewParamsList.size());
        const auto &viewParams = atlas.viewParamsList[patch.pduViewId()];

        // Look up depth value and affine parameters
        const auto uv = Vec2f(patch.atlasToView({j_atlas, i_atlas}));
        assert(atlas.geoFrame.getSize() == atlas.frameSize());
        auto level = atlas.geoFrame.getPlane(0)(i_atlas, j_atlas);

        const auto occupancyTransform = OccupancyTransform{viewParams, patch};
        if (!occupancyTransform.occupant(level)) {
          result.emplace_back();
          continue;
        }

        const auto d = depthTransform[patchId].expandDepth(level);
        assert(d > 0.F && isfinite(d));

        // Reproject and calculate ray angle
        const auto &R_t = transformList[patch.pduViewId()];
        const auto xyz = R_t(unprojectVertex(uv + Vec2f({0.5F, 0.5F}), d, viewParams.ci));
        const auto rayAngle = angle(xyz, xyz - R_t.translation());
        result.push_back({xyz, rayAngle});
      }
    }

    return result;
  }

  static auto atlasTriangles(const AtlasAccessUnit &atlas) {
    TriangleDescriptorList result;
    const auto rows = atlas.frameSize().y();
    const auto cols = atlas.frameSize().x();
    const int size = 2 * (rows - 1) * (cols - 1);
    result.reserve(size);

    auto addTriangle = [&](Vec2i v0, Vec2i v1, Vec2i v2) {
      const int patchId = atlas.patchId(v0.y(), v0.x());
      if (patchId == unusedPatchId || patchId != atlas.patchId(v1.y(), v1.x()) ||
          patchId != atlas.patchId(v2.y(), v2.x())) {
        return;
      }
      const auto vertexId0 = v0.y() * cols + v0.x();
      const auto vertexId1 = v1.y() * cols + v1.x();
      const auto vertexId2 = v2.y() * cols + v2.x();
      constexpr auto triangleArea = 0.5F;
      result.push_back({{vertexId0, vertexId1, vertexId2}, triangleArea});
    };

    for (int i = 1; i < rows; ++i) {
      for (int j = 1; j < cols; ++j) {
        const auto tl = Vec2i{j - 1, i - 1};
        const auto tr = Vec2i{j, i - 1};
        const auto bl = Vec2i{j - 1, i};
        const auto br = Vec2i{j, i};
        addTriangle(tl, tr, br);
        addTriangle(tl, br, bl);
      }
    }

    assert(int(result.size()) <= size);
    return result;
  }

  static auto atlasColors(const AtlasAccessUnit &atlas) {
    vector<Vec3f> result;
    auto yuv444 = expandTexture(atlas.attrFrame);
    result.reserve(distance(begin(result), end(result)));
    copy(begin(yuv444), end(yuv444), back_inserter(result));
    return result;
  }

  static auto unprojectAtlas(const AtlasAccessUnit &atlas, const ViewParams &viewportParams) {
    return tuple{atlasVertices(atlas, viewportParams), atlasTriangles(atlas),
                 tuple{atlasColors(atlas)}};
  }

  auto rasterFrame(const AccessUnit &frame, const ViewParams &viewportParams,
                   float compensation) const -> Rasterizer<Vec3f> {
    // Incremental view synthesis and blending
    Rasterizer<Vec3f> rasterizer{
        {m_rayAngleParam, m_depthParam, m_stretchingParam, m_maxStretching},
        viewportParams.ci.projectionPlaneSize()};

    // Pipeline mesh generation and rasterization
    future<void> runner = async(launch::deferred, []() {});

    for (const auto &atlas : frame.atlas) {
      // Generate a reprojected mesh
      auto [vertices, triangles, attributes] = unprojectAtlas(atlas, viewportParams);
      auto mesh = project(move(vertices), move(triangles), move(attributes), viewportParams.ci);

      // Compensate for resolution difference between source and target view
      for (auto &triangle : get<1>(mesh)) {
        triangle.area *= compensation;
      }

      // Synchronize with the rasterer
      runner.get();

      // Raster the mesh (asynchronously)
      runner = async(
          [&rasterizer](auto mesh) {
            rasterizer.submit(move(get<0>(mesh)), move(get<2>(mesh)), move(get<1>(mesh)));
            rasterizer.run();
          },
          move(mesh));
    }

    // Synchronize with the rasterer
    runner.get();
    return rasterizer;
  }

  // Field of view [rad]
  static auto xFoV(const ViewParams &viewParams) -> float {
    const auto &ci = viewParams.ci;
    return ci.dispatch(overload(
        [&](Equirectangular /*unused*/) { return abs(ci.ci_erp_phi_max() - ci.ci_erp_phi_min()); },
        [&](Perspective /*unused*/) {
          return 2.F * atan(ci.projectionPlaneSize().x() / (2 * ci.ci_perspective_focal_hor()));
        },
        [&](Orthographic /*unused*/) { return halfCycle; }));
  }

  // Resolution in px^2/rad^2
  static auto resolution(const ViewParams &viewParams) -> float {
    return square(viewParams.ci.projectionPlaneSize().x() / xFoV(viewParams));
  }

  static auto resolutionRatio(const AccessUnit &frame, const ViewParams &viewportParams) -> float {
    auto sum = 0.;
    auto count = 0;

    for (const auto &atlas : frame.atlas) {
      for (const auto &viewParams : atlas.viewParamsList) {
        sum += resolution(viewParams);
        ++count;
      }
    }
    return float(resolution(viewportParams) * count / sum);
  }

  auto renderFrame(const AccessUnit &frame, const ViewParams &viewportParams) const
      -> Texture444Depth16Frame {
    auto rasterizer = rasterFrame(frame, viewportParams, resolutionRatio(frame, viewportParams));

    const auto depthTransform = DepthTransform<16>{viewportParams.dq};
    auto viewport =
        Texture444Depth16Frame{quantizeTexture(rasterizer.attribute<0>()),
                               depthTransform.quantizeNormDisp(rasterizer.normDisp(), 1)};
    viewport.first.filIInvalidWithNeutral(viewport.second);

    return viewport;
  }

private:
  float m_rayAngleParam;
  float m_depthParam;
  float m_stretchingParam;
  float m_maxStretching;
}; // namespace TMIV::Renderer

AdditiveSynthesizer::AdditiveSynthesizer(const Json & /*rootNode*/, const Json &componentNode)
    : m_impl(new Impl(componentNode.require("rayAngleParameter").asFloat(),
                      componentNode.require("depthParameter").asFloat(),
                      componentNode.require("stretchingParameter").asFloat(),
                      componentNode.require("maxStretching").asFloat())) {}

AdditiveSynthesizer::AdditiveSynthesizer(float rayAngleParam, float depthParam,
                                         float stretchingParam, float maxStretching)
    : m_impl(new Impl(rayAngleParam, depthParam, stretchingParam, maxStretching)) {}

AdditiveSynthesizer::~AdditiveSynthesizer() = default;

auto AdditiveSynthesizer::renderFrame(const AccessUnit &frame,
                                      const ViewParams &viewportParams) const
    -> Texture444Depth16Frame {
  return m_impl->renderFrame(frame, viewportParams);
}
} // namespace TMIV::Renderer
