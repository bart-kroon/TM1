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

#include <TMIV/ViewOptimizer/AdditiveSynthesizer.h>

#include <TMIV/Common/LinAlg.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/Renderer/AffineTransform.h>
#include <TMIV/Renderer/Projector.h>
#include <TMIV/Renderer/Rasterizer.h>

#include <cmath>
#include <future>
#include <numeric>

namespace TMIV::ViewOptimizer {
using Renderer::AffineTransform;
using Renderer::ImageVertexDescriptorList;
using Renderer::Projector;
using Renderer::Rasterizer;
using Renderer::SceneVertexDescriptor;
using Renderer::SceneVertexDescriptorList;
using Renderer::TriangleDescriptorList;

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

  static auto atlasVertices(const SourceParams &params, size_t viewIdx,
                            const Common::DeepFrame &frame,
                            const MivBitstream::ViewParams &viewportParams) {
    SceneVertexDescriptorList result;
    const auto rows = frame.texture.getHeight();
    const auto cols = frame.texture.getWidth();
    result.reserve(rows * cols);

    const auto &viewParams = params.viewParamsList[viewIdx];
    const auto R_t = AffineTransform{viewParams.pose, viewportParams.pose};
    const auto depthTransform =
        MivBitstream::DepthTransform{viewParams.dq, frame.geometry.getBitDepth()};
    const auto occupancyTransform = MivBitstream::OccupancyTransform{viewParams};

    // Specialize on projection type
    viewParams.ci.dispatch([&](auto camType) {
      const auto projector = Projector<camType>{viewParams.ci};

      // For each used pixel in the atlas...
      for (int32_t i = 0; i < rows; ++i) {
        for (int32_t j = 0; j < cols; ++j) {
          // Look up depth value and affine parameters
          const auto uv = Common::Vec2f{Common::floatCast, Common::Vec2i{j, i}};
          const auto level = frame.geometry.getPlane(0)(i, j);

          // Skip non-occupant samples
          if (!occupancyTransform.occupant(level)) {
            result.emplace_back();
            continue;
          }

          const auto d = depthTransform.expandDepth(level);

          // Skip samples with invalid depth
          if (!(0.F < d)) {
            result.emplace_back();
            continue;
          }

          // Reproject and calculate ray angle
          const auto xyz = R_t(projector.unprojectVertex(uv + Common::Vec2f({0.5F, 0.5F}), d));
          const auto rayAngle = angle(xyz, xyz - R_t.translation());
          result.push_back({xyz, rayAngle});
        }
      }
    });

    return result;
  }

  static auto atlasTriangles(const Common::Vec2i &frameSize) {
    TriangleDescriptorList result;
    const auto rows = frameSize.y();
    const auto cols = frameSize.x();
    const int32_t size = 2 * (rows - 1) * (cols - 1);
    result.reserve(size);

    auto addTriangle = [&](Common::Vec2i v0, Common::Vec2i v1, Common::Vec2i v2) {
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

  static auto atlasColors(const Common::Frame<> &frame) {
    std::vector<Common::Vec3f> result;
    auto expandedFrame = expandTexture(yuv444(frame));
    result.reserve(distance(std::begin(result), std::end(result)));
    std::copy(std::begin(expandedFrame), std::end(expandedFrame), back_inserter(result));
    return result;
  }

  static auto unprojectView(const SourceParams &params, size_t viewIdx,
                            const Common::DeepFrame &frame,
                            const MivBitstream::ViewParams &viewportParams) {
    return std::tuple{atlasVertices(params, viewIdx, frame, viewportParams),
                      atlasTriangles(frame.texture.getSize()), atlasColors(frame.texture)};
  }

  // Project mesh to target view (equirectangular projection)
  static auto project(const Projector<MivBitstream::CiCamType::equirectangular> &projector,
                      const SceneVertexDescriptorList &sceneVertices,
                      TriangleDescriptorList triangles, std::vector<Common::Vec3f> color) {
    ImageVertexDescriptorList imageVertices;
    imageVertices.reserve(sceneVertices.size());
    for (const SceneVertexDescriptor &v : sceneVertices) {
      imageVertices.push_back(projector.projectVertex(v));
    }

    // Weighted sphere compensation of stretching
    for (auto &triangle : triangles) {
      auto v = 0.F;
      for (auto index : triangle.indices) {
        v += imageVertices[index].position.y() / 3.F;
      }
      triangle.area /= projector.weightedSphere(v);
    }

    return std::tuple{std::move(imageVertices), std::move(triangles), std::move(color)};
  }

  // Project mesh to target view (perspective projection)
  static auto project(const Projector<MivBitstream::CiCamType::perspective> &projector,
                      const SceneVertexDescriptorList &sceneVertices,
                      TriangleDescriptorList triangles, std::vector<Common::Vec3f> color) {
    ImageVertexDescriptorList imageVertices;
    imageVertices.reserve(sceneVertices.size());
    for (const SceneVertexDescriptor &v : sceneVertices) {
      imageVertices.push_back(projector.projectVertex(v));
    }
    return std::tuple{std::move(imageVertices), std::move(triangles), std::move(color)};
  }

  // Project mesh to target view (orthographic projection)
  static auto project(const Projector<MivBitstream::CiCamType::orthographic> &projector,
                      const SceneVertexDescriptorList &sceneVertices,
                      TriangleDescriptorList triangles, std::vector<Common::Vec3f> color) {
    ImageVertexDescriptorList imageVertices;
    imageVertices.reserve(sceneVertices.size());
    for (const SceneVertexDescriptor &v : sceneVertices) {
      imageVertices.push_back(projector.projectVertex(v));
    }
    return std::tuple{std::move(imageVertices), std::move(triangles), std::move(color)};
  }

  // Project mesh to target view (dispatch on projection type)
  static auto project(SceneVertexDescriptorList vertices, TriangleDescriptorList triangles,
                      std::vector<Common::Vec3f> color,
                      const MivBitstream::CameraIntrinsics &target) {
    return target.dispatch([&](auto camType) {
      Projector<camType> projector{target};
      return project(projector, std::move(vertices), std::move(triangles), std::move(color));
    });
  }

  [[nodiscard]] auto rasterFrame(const SourceParams &params, const Common::DeepFrameList &frame,
                                 const MivBitstream::ViewParams &viewportParams,
                                 float compensation) const -> Rasterizer {
    // Incremental view synthesis and blending
    Rasterizer rasterizer{{m_rayAngleParam, m_depthParam, m_stretchingParam, m_maxStretching},
                          viewportParams.ci.projectionPlaneSize()};

    // Pipeline mesh generation and rasterization
    std::future<void> runner = std::async(std::launch::deferred, []() {});

    for (size_t viewIdx = 0; viewIdx < frame.size(); ++viewIdx) {
      // Generate a reprojected mesh
      auto [vertices, triangles, color] =
          unprojectView(params, viewIdx, frame[viewIdx], viewportParams);
      auto mesh =
          project(std::move(vertices), std::move(triangles), std::move(color), viewportParams.ci);

      // Compensate for resolution difference between source and target view
      for (auto &triangle : std::get<1>(mesh)) {
        triangle.area *= compensation;
      }

      // Synchronize with the rasterer
      runner.get();

      // Raster the mesh (asynchronously)
      runner = async(
          [&rasterizer](auto mesh_) {
            rasterizer.submit(std::move(std::get<0>(mesh_)), std::move(std::get<2>(mesh_)),
                              std::move(std::get<1>(mesh_)));
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

  static auto resolutionRatio(const SourceParams &params,
                              const MivBitstream::ViewParams &viewportParams) -> float {
    auto sum = 0.;
    auto count = 0.;

    for (const auto &viewParams : params.viewParamsList) {
      sum += resolution(viewParams);
      count += 1.;
    }
    return static_cast<float>(resolution(viewportParams) * count / sum);
  }

  [[nodiscard]] auto renderFrame(const SourceParams &params, const Common::DeepFrameList &frame,
                                 const MivBitstream::CameraConfig &cameraConfig) const
      -> Common::RendererFrame {
    auto rasterizer = rasterFrame(params, frame, cameraConfig.viewParams,
                                  resolutionRatio(params, cameraConfig.viewParams));

    const auto depthTransform =
        MivBitstream::DepthTransform{cameraConfig.viewParams.dq, cameraConfig.bitDepthGeometry};
    auto viewport = Common::RendererFrame{
        Common::quantizeTexture(rasterizer.color(), cameraConfig.bitDepthTexture),
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

AdditiveSynthesizer::AdditiveSynthesizer(const Common::Json & /* rootNode */,
                                         const Common::Json &componentNode)
    : m_impl(new Impl(componentNode.require("rayAngleParameter").as<float>(),
                      componentNode.require("depthParameter").as<float>(),
                      componentNode.require("stretchingParameter").as<float>(),
                      componentNode.require("maxStretching").as<float>())) {}

AdditiveSynthesizer::~AdditiveSynthesizer() = default;

auto AdditiveSynthesizer::renderFrame(const SourceParams &params,
                                      const Common::DeepFrameList &frame,
                                      const MivBitstream::CameraConfig &cameraConfig) const
    -> Common::RendererFrame {
  return m_impl->renderFrame(params, frame, cameraConfig);
}
} // namespace TMIV::ViewOptimizer
