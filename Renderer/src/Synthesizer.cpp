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

#include <cassert>

#include "AccumulatingView.h"
#include <TMIV/Image/Image.h>
#include <TMIV/Renderer/reprojectPoints.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::Image;

namespace TMIV::Renderer {
namespace {
WrappingMethod wrappingMethod(const CameraParameters &camera) {
  if (camera.type == ProjectionType::ERP &&
      camera.erpPhiRange == Vec2f{-180.f, 180.f}) {
    return WrappingMethod::horizontal;
  }
  return WrappingMethod::none;
}
} // namespace

Synthesizer::Synthesizer(const Common::Json &node) {
  if (auto subnode = node.optional("RayAngleParam"))
    m_rayAngleParam = subnode.asFloat();

  if (auto subnode = node.optional("DepthParam"))
    m_depthParam = subnode.asFloat();

  if (auto subnode = node.optional("StretchingParam"))
    m_stretchingParam = subnode.asFloat();
}

Synthesizer::Synthesizer(double rayAngleParam, double depthParam,
                         double stretchingParam)
    : m_rayAngleParam{rayAngleParam}, m_depthParam{depthParam},
      m_stretchingParam{stretchingParam} {}

Common::TextureDepth10Frame
Synthesizer::renderFrame(const Common::MVD10Frame &atlas,
                         const Common::PatchIdMapList &maps,
                         const Metadata::PatchParameterList &patches,
                         const Metadata::CameraParameterList &cameras,
                         const Metadata::CameraParameters &target) const {
  return {};
}

Common::TextureDepth16Frame
Synthesizer::renderFrame(const Common::MVD16Frame &atlas,
                         const Metadata::CameraParameterList &cameras,
                         const Metadata::CameraParameters &target) const {
  AccumulatingView av{m_rayAngleParam, m_depthParam, m_stretchingParam,
                      AccumulatingPixel::Mode::all};

  assert(atlas.size() == cameras.size());
  auto i_camera = begin(cameras);

  for (const auto &view : atlas) {
    auto &camera = *i_camera++;

    auto outPoints =
        changeReferenceFrame(camera, target,
                             unprojectPoints(camera, imagePositions(camera),
                                             expandDepth(camera, view.second)));
    auto positions_depth = projectPoints(target, outPoints);
    auto rayAngles = calculateRayAngles(camera, target, outPoints);

    av.transform(expandTexture(view.first), positions_depth.first,
                 positions_depth.second, rayAngles, target.size,
                 wrappingMethod(target));
  }

  return {quantizeTexture(av.texture()),
          quantizeNormDisp16(target, av.normDisp())};
}

Mat1f Synthesizer::renderDepth(const Mat1f &frame,
                               const CameraParameters &camera,
                               const CameraParameters &target) const {
  AccumulatingView av{m_rayAngleParam, m_depthParam, m_stretchingParam,
                      AccumulatingPixel::Mode::depth};

  auto outPoints = changeReferenceFrame(
      camera, target, unprojectPoints(camera, imagePositions(camera), frame));
  auto positions_depth = projectPoints(target, outPoints);
  auto rayAngles = calculateRayAngles(camera, target, outPoints);

  // TODO: Templatize AccumulatingView to avoid interpolating and alphablending
  // texture for nothing
  Mat3f texture{positions_depth.first.sizes()};
  av.transform(texture, positions_depth.first, positions_depth.second,
               rayAngles, target.size, wrappingMethod(target));
  return av.depth();
}
} // namespace TMIV::Renderer
