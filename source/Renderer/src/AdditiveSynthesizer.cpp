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

#include <TMIV/Renderer/AdditiveSynthesizer.h>

#include <TMIV/Common/LinAlg.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/Renderer/Engine.h>
#include <TMIV/Renderer/Rasterizer.h>
#include <TMIV/Renderer/reprojectPoints.h>

#include <cmath>
#include <future>
#include <numeric>

namespace TMIV::Renderer {
class AdditiveSynthesizer::Impl {
public:
  Impl(float rayAngleParam, float depthParam, float stretchingParam, float maxStretching)
      : m_rayAngleParam{rayAngleParam}
      , m_depthParam{depthParam}
      , m_stretchingParam{stretchingParam}
      , m_maxStretching{maxStretching} {}

  Impl(const Impl &) = delete;
  Impl(Impl &&) = delete;
  auto operator=(const Impl &) -> Impl & = delete;
  auto operator=(Impl &&) -> Impl & = delete;
  ~Impl() = default;

  static auto affineTransformList(const MivBitstream::ViewParamsList &viewParamsList,
                                  const MivBitstream::Pose &target) {
    std::vector<AffineTransform> result;
    result.reserve(viewParamsList.size());
    for (const auto &source : viewParamsList) {
      result.emplace_back(source.pose, target);
    }
    return result;
  }

  static auto atlasVertices(const MivBitstream::AccessUnit &frame,
                            const MivBitstream::AtlasAccessUnit &atlas,
                            const MivBitstream::ViewParams &viewportParams) {
    SceneVertexDescriptorList result;
    const auto rows = atlas.texFrame.getHeight();
    const auto cols = atlas.texFrame.getWidth();
    result.reserve(rows * cols);

    const auto transformList = affineTransformList(frame.viewParamsList, viewportParams.pose);

    std::vector<MivBitstream::DepthTransform> depthTransform;
    depthTransform.reserve(atlas.patchParamsList.size());

    for (const auto &patch : atlas.patchParamsList) {
      const auto geoBitDepth = atlas.geoFrame.getBitDepth();
      depthTransform.emplace_back(frame.viewParamsList[patch.atlasPatchProjectionId()].dq, patch,
                                  geoBitDepth);
    }

    // For each used pixel in the atlas...
    for (int32_t i_atlas = 0; i_atlas < rows; ++i_atlas) {
      for (int32_t j_atlas = 0; j_atlas < cols; ++j_atlas) {
        const auto patchIdx = atlas.filteredPatchIdx(i_atlas, j_atlas);

        // Push dummy vertices to keep indexing simple
        if (patchIdx == Common::unusedPatchIdx) {
          result.emplace_back();
          continue;
        }

        // Exclude non-occupant pixels
        if (!atlas.occFrame.getPlane(0)(i_atlas, j_atlas)) {
          result.emplace_back();
          continue;
        }

        // Look up metadata
        const auto &patch = atlas.patchParamsList[patchIdx];
        const auto viewIdx = frame.viewParamsList.indexOf(patch.atlasPatchProjectionId());
        const auto &viewParams = frame.viewParamsList[viewIdx];

        // Look up depth value and affine parameters
        const auto uv = Common::Vec2f{Common::floatCast, patch.atlasToView({j_atlas, i_atlas})};
        auto level = atlas.geoFrame.getPlane(0)(i_atlas, j_atlas);
        const auto d = depthTransform[patchIdx].expandDepth(level);

        // Reproject and calculate ray angle
        const auto &R_t = transformList[viewIdx];
        const auto xyz = R_t(unprojectVertex(uv + Common::Vec2f({0.5F, 0.5F}), d, viewParams.ci));
        const auto rayAngle = angle(xyz, xyz - R_t.translation());
        result.push_back({xyz, rayAngle});
      }
    }

    return result;
  }

  static auto atlasTriangles(const MivBitstream::AtlasAccessUnit &atlas) {
    TriangleDescriptorList result;
    const auto rows = atlas.texFrame.getHeight();
    const auto cols = atlas.texFrame.getWidth();
    const int32_t size = 2 * (rows - 1) * (cols - 1);
    result.reserve(size);

    auto addTriangle = [&](Common::Vec2i v0, Common::Vec2i v1, Common::Vec2i v2) {
      const int32_t patchIdx = atlas.filteredPatchIdx(v0.y(), v0.x());
      if (patchIdx == Common::unusedPatchIdx ||
          patchIdx != atlas.filteredPatchIdx(v1.y(), v1.x()) ||
          patchIdx != atlas.filteredPatchIdx(v2.y(), v2.x())) {
        return;
      }
      const auto vertexId0 = v0.y() * cols + v0.x();
      const auto vertexId1 = v1.y() * cols + v1.x();
      const auto vertexId2 = v2.y() * cols + v2.x();
      constexpr auto triangleArea = 0.5F;
      result.push_back({{vertexId0, vertexId1, vertexId2}, triangleArea});
    };

    for (int32_t i = 1; i < rows; ++i) {
      for (int32_t j = 1; j < cols; ++j) {
        const auto tl = Common::Vec2i{j - 1, i - 1};
        const auto tr = Common::Vec2i{j, i - 1};
        const auto bl = Common::Vec2i{j - 1, i};
        const auto br = Common::Vec2i{j, i};
        addTriangle(tl, tr, br);
        addTriangle(tl, br, bl);
      }
    }

    POSTCONDITION(static_cast<int32_t>(result.size()) <= size);
    return result;
  }

  static auto atlasColors(const MivBitstream::AtlasAccessUnit &atlas) {
    std::vector<Common::Vec3f> result;
    auto yuv444 = expandTexture(atlas.texFrame);
    result.reserve(distance(std::begin(result), std::end(result)));
    std::copy(std::begin(yuv444), std::end(yuv444), back_inserter(result));
    return result;
  }

  static auto unprojectAtlas(const MivBitstream::AccessUnit &frame,
                             const MivBitstream::AtlasAccessUnit &atlas,
                             const MivBitstream::ViewParams &viewportParams) {
    return std::tuple{atlasVertices(frame, atlas, viewportParams), atlasTriangles(atlas),
                      std::tuple{atlasColors(atlas)}};
  }

  [[nodiscard]] auto rasterFrame(const MivBitstream::AccessUnit &frame,
                                 const MivBitstream::ViewParams &viewportParams,
                                 float compensation) const -> Rasterizer<Common::Vec3f> {
    // Incremental view synthesis and blending
    Rasterizer<Common::Vec3f> rasterizer{
        {m_rayAngleParam, m_depthParam, m_stretchingParam, m_maxStretching},
        viewportParams.ci.projectionPlaneSize()};

    // Pipeline mesh generation and rasterization
    std::future<void> runner = std::async(std::launch::deferred, []() {});

    for (const auto &atlas : frame.atlas) {
      if (atlas.asps.asps_miv_extension_present_flag() &&
          atlas.asps.asps_miv_extension().asme_ancillary_atlas_flag()) {
        continue;
      }

      // Generate a reprojected mesh
      auto [vertices, triangles, attributes] = unprojectAtlas(frame, atlas, viewportParams);
      auto mesh = project(std::move(vertices), std::move(triangles), std::move(attributes),
                          viewportParams.ci);

      // Compensate for resolution difference between source and target view
      for (auto &triangle : std::get<1>(mesh)) {
        triangle.area *= compensation;
      }

      // Synchronize with the rasterer
      runner.get();

      // Raster the mesh (asynchronously)
      runner = async(
          [&rasterizer](auto mesh) {
            rasterizer.submit(std::move(std::get<0>(mesh)), std::move(std::get<2>(mesh)),
                              std::move(std::get<1>(mesh)));
            rasterizer.run();
          },
          std::move(mesh));
    }

    // Synchronize with the rasterer
    runner.get();
    return rasterizer;
  }

  // Field of view [rad]
  static auto xFoV(const MivBitstream::ViewParams &viewParams) -> float {
    const auto &ci = viewParams.ci;
    return ci.dispatch(Common::overload(
        [&](MivBitstream::Equirectangular /*unused*/) {
          return Common::deg2rad(std::abs(ci.ci_erp_phi_max() - ci.ci_erp_phi_min()));
        },
        [&](MivBitstream::Perspective /*unused*/) {
          return 2.F *
                 std::atan(ci.projectionPlaneSizeF().x() / (2 * ci.ci_perspective_focal_hor()));
        },
        [&](MivBitstream::Orthographic /*unused*/) { return Common::pi<float>; }));
  }

  // Resolution in px^2/rad^2
  static auto resolution(const MivBitstream::ViewParams &viewParams) -> float {
    return Common::sqr(viewParams.ci.projectionPlaneSizeF().x() / xFoV(viewParams));
  }

  static auto resolutionRatio(const MivBitstream::AccessUnit &frame,
                              const MivBitstream::ViewParams &viewportParams) -> float {
    auto sum = 0.;
    auto count = 0.;

    for (const auto &viewParams : frame.viewParamsList) {
      sum += resolution(viewParams);
      count += 1.;
    }
    return static_cast<float>(resolution(viewportParams) * count / sum);
  }

  [[nodiscard]] auto renderFrame(const MivBitstream::AccessUnit &frame,
                                 const MivBitstream::CameraConfig &cameraConfig) const
      -> Common::RendererFrame {
    auto rasterizer = rasterFrame(frame, cameraConfig.viewParams,
                                  resolutionRatio(frame, cameraConfig.viewParams));

    const auto depthTransform =
        MivBitstream::DepthTransform{cameraConfig.viewParams.dq, cameraConfig.bitDepthGeometry};
    auto viewport = Common::RendererFrame{
        Common::quantizeTexture(rasterizer.attribute<0>(), cameraConfig.bitDepthTexture),
        depthTransform.quantizeNormDisp(rasterizer.normDisp(), 1)};
    viewport.texture.fillInvalidWithNeutral(viewport.geometry);

    return viewport;
  }

private:
  float m_rayAngleParam;
  float m_depthParam;
  float m_stretchingParam;
  float m_maxStretching;
}; // namespace TMIV::Renderer

AdditiveSynthesizer::AdditiveSynthesizer(const Common::Json & /*rootNode*/,
                                         const Common::Json &componentNode)
    : m_impl(new Impl(componentNode.require("rayAngleParameter").as<float>(),
                      componentNode.require("depthParameter").as<float>(),
                      componentNode.require("stretchingParameter").as<float>(),
                      componentNode.require("maxStretching").as<float>())) {}

AdditiveSynthesizer::AdditiveSynthesizer(float rayAngleParam, float depthParam,
                                         float stretchingParam, float maxStretching)
    : m_impl(new Impl(rayAngleParam, depthParam, stretchingParam, maxStretching)) {}

AdditiveSynthesizer::~AdditiveSynthesizer() = default;

auto AdditiveSynthesizer::renderFrame(const MivBitstream::AccessUnit &frame,
                                      const MivBitstream::CameraConfig &cameraConfig) const
    -> Common::RendererFrame {
  return m_impl->renderFrame(frame, cameraConfig);
}
} // namespace TMIV::Renderer
