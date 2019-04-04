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

#include "AccumulatingView.h"
#include <TMIV/Renderer/reprojectPoints.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {
namespace {
// TODO: Make a better mesh. This is just to get started.
Mat2f imagePositions(const CameraParameters &camera) {
  Mat2f result;
  result.resize(camera.size.y(), camera.size.x());
  for (unsigned i = 0; i != result.height(); ++i) {
    for (unsigned j = 0; j != result.width(); ++j) {
      result(i, j) = {float(j) + 0.5f, float(i) + 0.5f};
    }
  }
  return result;
}

WrappingMethod wrappingMethod(const CameraParameters &camera) {
  if (camera.type == ProjectionType::ERP &&
      camera.erpPhiRange == Vec2f{-180.f, 180.f}) {
    return WrappingMethod::horizontal;
  }
  return WrappingMethod::none;
}
} // namespace

TextureFrame Synthesizer::renderTexture(const MVDFrame &frame,
                                        const PatchParameterList &patches,
                                        const CameraParameterList &cameras,
                                        const CameraParameters &target) const {
  return {};
}

DepthFrame Synthesizer::renderDepth(const MVDFrame &frame,
                                    const PatchParameterList &patches,
                                    const CameraParameterList &cameras,
                                    const CameraParameters &target) const {
  return {};
}

TextureDepthFrame Synthesizer::renderTextureDepth(
    const MVDFrame &frame, const PatchParameterList &patches,
    const CameraParameterList &cameras, const CameraParameters &target) const {
  return {};
}

Mat<float> Synthesizer::renderDepth(const Mat1f &frame,
                                    const CameraParameters &camera,
                                    const CameraParameters &target) const {
  auto [positions, depth] =
      reprojectPoints(camera, target, imagePositions(camera), frame);

  AccumulatingView av{m_rayAngleParam, m_depthParam, m_stretchingParam};

  // TODO: Templatize AccumulatingView to avoid interpolating and alphablending
  // texture for nothing
  Mat3f texture{depth.sizes()};
  av.transform(texture, positions, depth, target.size, wrappingMethod(target));
  return av.depth();
}
} // namespace TMIV::Renderer