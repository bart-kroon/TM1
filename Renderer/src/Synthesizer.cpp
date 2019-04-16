/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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
#include <TMIV/Image/Image.h>
#include <cassert>
#include <future>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::Image;

namespace TMIV::Renderer {
class Synthesizer::Impl {
public:
  Impl(float rayAngleParam, float depthParam, float stretchingParam)
      : m_rayAngleParam{rayAngleParam}, m_depthParam{depthParam},
        m_stretchingParam{stretchingParam} {}

  Impl(const Impl &) = delete;
  Impl(Impl &&) = delete;
  Impl &operator=(const Impl &) = delete;
  Impl &operator=(Impl &&) = delete;

  tuple<SceneVertexDescriptorList, TriangleDescriptorList, tuple<vector<Vec3f>>>
  unprojectAtlas(const TextureDepth10Frame &atlas, const PatchIdMap &map,
                 const PatchParameterList &patches,
                 const CameraParameterList &cameras,
                 const CameraParameters &target) const {
    // TODO...
    return {};
  }

  template <typename Unprojector>
  Rasterizer<Vec3f> rasterFrame(size_t numViews, const CameraParameters &target,
                                Unprojector unprojector) const {
    // Incremental view synthesis and blending
    Rasterizer<Vec3f> rasterizer{
        {m_rayAngleParam, m_depthParam, m_stretchingParam}, target.size};

    // Pipeline mesh generation and rasterization
    future<void> runner = async(launch::deferred, []() {});

    for (size_t i = 0; i < numViews; ++i) {
      // Generate a reprojected mesh
      auto [vertices, triangles, attributes] = unprojector(i, target);
      auto mesh =
          project(move(vertices), move(triangles), move(attributes), target);

      // Synchronize with the rasterer
      runner.get();

      // Raster the mesh (asynchronously)
      runner = async(
          [&rasterizer](auto mesh) {
            rasterizer.submit(move(get<0>(mesh)), move(get<2>(mesh)),
                              move(get<1>(mesh)));
            rasterizer.run();
          },
          move(mesh));
    }

    // Synchronize with the rasterer
    runner.get();
    return rasterizer;
  }

  TextureDepth10Frame renderFrame(const MVD10Frame &atlases,
                                  const PatchIdMapList &maps,
                                  const PatchParameterList &patches,
                                  const CameraParameterList &cameras,
                                  const CameraParameters &target) const {
    assert(atlases.size() == maps.size());
    auto rasterizer = rasterFrame(
        atlases.size(), target, [&](size_t i, const CameraParameters &target) {
          return unprojectAtlas(atlases[i], maps[i], patches, cameras, target);
        });
    return {quantizeTexture(rasterizer.attribute<0>()),
            quantizeNormDisp10(target, rasterizer.normDisp())};
  }

  TextureDepth16Frame renderFrame(const MVD16Frame &frame,
                                  const CameraParameterList &cameras,
                                  const CameraParameters &target) const {
    assert(frame.size() == cameras.size());
    auto rasterizer = rasterFrame(
        frame.size(), target, [&](size_t i, const CameraParameters &target) {
          return unproject(expandDepth(cameras[i], frame[i].second), cameras[i],
                           target, expandTexture(frame[i].first));
        });
    return {quantizeTexture(rasterizer.attribute<0>()),
            quantizeNormDisp16(target, rasterizer.normDisp())};
  }

  Mat<float> renderDepth(const Mat<float> &depth,
                         const CameraParameters &camera,
                         const CameraParameters &target) const {
    AccumulatingPixel<> pixel{m_rayAngleParam, m_depthParam, m_stretchingParam};
    auto mesh = reproject(depth, camera, target);

    Rasterizer<> rasterizer{pixel, target.size};
    rasterizer.submit(move(get<0>(mesh)), {}, move(get<1>(mesh)));
    rasterizer.run();
    return rasterizer.depth();
  }

private:
  float m_rayAngleParam;
  float m_depthParam;
  float m_stretchingParam;
};

Synthesizer::Synthesizer(const Common::Json &node)
    : m_impl(new Impl(
          node.require("Synthesizer").require("rayAngleParam").asFloat(),
          node.require("Synthesizer").require("depthParam").asFloat(),
          node.require("Synthesizer").require("stretchingParam").asFloat())) {}

Synthesizer::Synthesizer(float rayAngleParam, float depthParam,
                         float stretchingParam)
    : m_impl(new Impl(rayAngleParam, depthParam, stretchingParam)) {}

Synthesizer::~Synthesizer() {}

Common::TextureDepth10Frame
Synthesizer::renderFrame(const Common::MVD10Frame &atlas,
                         const Common::PatchIdMapList &maps,
                         const Metadata::PatchParameterList &patches,
                         const Metadata::CameraParameterList &cameras,
                         const Metadata::CameraParameters &target) const {
  return m_impl->renderFrame(atlas, maps, patches, cameras, target);
}

Common::TextureDepth16Frame
Synthesizer::renderFrame(const Common::MVD16Frame &frame,
                         const Metadata::CameraParameterList &cameras,
                         const Metadata::CameraParameters &target) const {
  return m_impl->renderFrame(frame, cameras, target);
}

Common::Mat<float>
Synthesizer::renderDepth(const Common::Mat<float> &frame,
                         const Metadata::CameraParameters &camera,
                         const Metadata::CameraParameters &target) const {
  return m_impl->renderDepth(frame, camera, target);
}
} // namespace TMIV::Renderer
