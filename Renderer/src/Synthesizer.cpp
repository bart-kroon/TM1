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

#include <TMIV/Renderer/Synthesizer.h>

#include "Engine.h"
#include "Rasterizer.h"
#include <TMIV/Common/LinAlg.h>
#include <TMIV/Image/Image.h>
#include <cassert>
#include <cmath>
#include <future>
#include <numeric>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::Image;

namespace TMIV::Renderer {
class Synthesizer::Impl {
public:
  Impl(float rayAngleParam, float depthParam, float stretchingParam, float maxStretching)
      : m_rayAngleParam{rayAngleParam}, m_depthParam{depthParam},
        m_stretchingParam{stretchingParam}, m_maxStretching{maxStretching} {}

  Impl(const Impl &) = delete;
  Impl(Impl &&) = delete;
  Impl &operator=(const Impl &) = delete;
  Impl &operator=(Impl &&) = delete;
  ~Impl() = default;

  auto affineParameterList(const CameraParametersList &cameras,
                           const CameraParameters &target) const {
    vector<pair<Mat3x3f, Vec3f>> result;
    result.reserve(cameras.size());
    transform(
        begin(cameras), end(cameras), back_inserter(result),
        [&target](const CameraParameters &camera) { return affineParameters(camera, target); });
    return result;
  }

  auto atlasVertices(const TextureDepth16Frame &atlas, const Mat<uint16_t> &ids,
                     const AtlasParametersList &patches, const CameraParametersList &cameras,
                     const CameraParameters &target) const {
    SceneVertexDescriptorList result;
    const auto rows = int(ids.height());
    const auto cols = int(ids.width());
    result.reserve(rows * cols);

    auto R_t = affineParameterList(cameras, target);

    auto i_ids = begin(ids);

    // For each used pixel in the atlas...
    for (int i_atlas = 0; i_atlas < rows; ++i_atlas) {
      for (int j_atlas = 0; j_atlas < cols; ++j_atlas) {
        auto patchId = *i_ids++;

        // Push dummy vertices to keep indexing simple
        if (patchId == unusedPatchId) {
          result.emplace_back();
          continue;
        }

        // Look up metadata
        assert(patchId < patches.size());
        const auto &patch = patches[patchId];
        assert(patch.viewId < cameras.size());
        const auto &camera = cameras[patch.viewId];

        // Look up depth value and affine parameters
        const auto uv = Vec2f(atlasToView({j_atlas, i_atlas}, patch));

        auto d16 = atlas.second.getPlane(0)(i_atlas, j_atlas);
        assert(d16 > 0U); // #29: Having a patch ID, this depth has to be valid.
        const auto d = expandDepthValue<16>(camera, d16);
        const auto &R = R_t[patch.viewId].first;
        const auto &t = R_t[patch.viewId].second;

        // Reproject and calculate ray angle
        const auto xyz = R * unprojectVertex(uv + Vec2f({0.5F, 0.5F}), d, camera) + t;
        const auto rayAngle = angle(xyz, xyz - t);
        result.push_back({xyz, rayAngle});
      }
    }

    return result;
  }

  auto atlasTriangles(const Mat<uint16_t> &ids) const {
    TriangleDescriptorList result;
    const int rows = int(ids.height());
    const int cols = int(ids.width());
    const int size = 2 * (rows - 1) * (cols - 1);
    result.reserve(size);

    auto addTriangle = [&result, &ids](int v0, int v1, int v2) {
      const int id0 = ids[v0];
      if (id0 == unusedPatchId || id0 != ids[v1] || id0 != ids[v2]) {
        return;
      }
      constexpr auto triangleArea = 0.5F;
      result.push_back({{v0, v1, v2}, triangleArea});
    };

    for (int i = 1; i < rows; ++i) {
      for (int j = 1; j < cols; ++j) {
        const int tl = (i - 1) * cols + (j - 1);
        const int tr = (i - 1) * cols + j;
        const int bl = i * cols + (j - 1);
        const int br = i * cols + j;
        addTriangle(tl, tr, br);
        addTriangle(tl, br, bl);
      }
    }

    assert(int(result.size()) <= size);
    return result;
  }

  auto atlasColors(const TextureDepth16Frame &atlas) const {
    vector<Vec3f> result;
    auto yuv444 = expandTexture(atlas.first);
    result.reserve(distance(begin(result), end(result)));
    copy(begin(yuv444), end(yuv444), back_inserter(result));
    return result;
  }

  auto unprojectAtlas(const TextureDepth16Frame &atlas, const Mat<uint16_t> &ids,
                      const AtlasParametersList &patches, const CameraParametersList &cameras,
                      const CameraParameters &target) const {
    assert(int(ids.height()) == atlas.first.getHeight());
    assert(int(ids.height()) == atlas.second.getHeight());
    assert(int(ids.width()) == atlas.first.getWidth());
    assert(int(ids.width()) == atlas.second.getWidth());
    return tuple{atlasVertices(atlas, ids, patches, cameras, target), atlasTriangles(ids),
                 tuple{atlasColors(atlas)}};
  }

  template <typename Unprojector>
  Rasterizer<Vec3f> rasterFrame(size_t numViews, const CameraParameters &target,
                                Unprojector unprojector, float compensation) const {
    // Incremental view synthesis and blending
    Rasterizer<Vec3f> rasterizer{
        {m_rayAngleParam, m_depthParam, m_stretchingParam, m_maxStretching}, target.size};

    // Pipeline mesh generation and rasterization
    future<void> runner = async(launch::deferred, []() {});

    for (size_t i = 0; i < numViews; ++i) {
      // Generate a reprojected mesh
      auto [vertices, triangles, attributes] = unprojector(i, target);
      auto mesh = project(move(vertices), move(triangles), move(attributes), target);

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

  // Field of view in deg
  static float xFoV(const CameraParameters &camera) {
    switch (camera.type) {
    case ProjectionType::ERP:
      return abs(camera.erpPhiRange[1] - camera.erpPhiRange[0]);
    case ProjectionType::Perspective:
      return degperrad * 2 * atan(camera.size.x() / (2 * camera.perspectiveFocal.x()));
    default:
      return fullCycle;
    }
  }

  // Resolution in px^2/deg^2
  static float resolution(const CameraParameters &camera) {
    return square(camera.size.x() / xFoV(camera));
  }

  static float resolutionRatio(const CameraParametersList &cameras,
                               const CameraParameters &target) {
    const auto sourceResolution = accumulate(begin(cameras), end(cameras), 0.F,
                                             [&](float average, const CameraParameters &camera) {
                                               return average + resolution(camera) / cameras.size();
                                             });
    return resolution(target) / sourceResolution;
  }

   Texture444Depth16Frame renderFrame(const MVD16Frame &atlases, const PatchIdMapList &ids,
                                     const AtlasParametersList &patches,
                                     const CameraParametersList &cameras,
                                     const CameraParameters &target) const {
    assert(atlases.size() == ids.size());
    auto rasterizer = rasterFrame(
        atlases.size(), target,
        [&](size_t i, const CameraParameters &target) {
          return unprojectAtlas(atlases[i], ids[i].getPlane(0), patches, cameras, target);
        },
        resolutionRatio(cameras, target));
    return {quantizeTexture(rasterizer.attribute<0>()),
            quantizeNormDisp16(target, rasterizer.normDisp())};
  }

  Texture444Depth16Frame renderFrame(const MVD16Frame &frame, const CameraParametersList &cameras,
                                     const CameraParameters &target) const {
    assert(frame.size() == cameras.size());
    auto rasterizer = rasterFrame(
        frame.size(), target,
        [&](size_t i, const CameraParameters &target) {
          return unproject(expandDepth(cameras[i], frame[i].second), cameras[i], target,
                           expandTexture(frame[i].first));
        },
        resolutionRatio(cameras, target));
    return {quantizeTexture(rasterizer.attribute<0>()),
            quantizeNormDisp16(target, rasterizer.normDisp())};
  }

  Mat<float> renderDepth(const Mat<float> &depth, const CameraParameters &camera,
                         const CameraParameters &target) const {
    AccumulatingPixel<> pixel{m_rayAngleParam, m_depthParam, m_stretchingParam, m_maxStretching};
    auto mesh = reproject(depth, camera, target);

    Rasterizer<> rasterizer{pixel, target.size};
    rasterizer.submit(move(get<0>(mesh)), {}, get<1>(mesh));
    rasterizer.run();
    return rasterizer.depth();
  }

private:
  float m_rayAngleParam;
  float m_depthParam;
  float m_stretchingParam;
  float m_maxStretching;
}; // namespace TMIV::Renderer

Synthesizer::Synthesizer(const Common::Json & /*rootNode*/, const Common::Json &componentNode)
    : m_impl(new Impl(componentNode.require("rayAngleParameter").asFloat(),
                      componentNode.require("depthParameter").asFloat(),
                      componentNode.require("stretchingParameter").asFloat(),
                      componentNode.require("maxStretching").asFloat())) {}

Synthesizer::Synthesizer(float rayAngleParam, float depthParam, float stretchingParam,
                         float maxStretching)
    : m_impl(new Impl(rayAngleParam, depthParam, stretchingParam, maxStretching)) {}

Synthesizer::~Synthesizer() = default;

Common::Texture444Depth16Frame Synthesizer::renderFrame(const Common::MVD16Frame &atlas,
                                                        const Common::PatchIdMapList &maps,
                                                        const AtlasParametersList &patches,
                                                        const CameraParametersList &cameras,
                                                        const CameraParameters &target) const {
  return m_impl->renderFrame(atlas, maps, patches, cameras, target);
}

Common::Texture444Depth16Frame Synthesizer::renderFrame(const Common::MVD16Frame &frame,
                                                        const CameraParametersList &cameras,
                                                        const CameraParameters &target) const {
  return m_impl->renderFrame(frame, cameras, target);
}

Common::Mat<float> Synthesizer::renderDepth(const Common::Mat<float> &frame,
                                            const CameraParameters &camera,
                                            const CameraParameters &target) const {
  return m_impl->renderDepth(frame, camera, target);
}
} // namespace TMIV::Renderer
