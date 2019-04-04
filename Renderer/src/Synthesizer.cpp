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
#include <TMIV/Renderer/reprojectPoints.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {
namespace {
WrappingMethod wrappingMethod(const CameraParameters &camera) {
  if (camera.type == ProjectionType::ERP &&
      camera.erpPhiRange == Vec2f{-180.f, 180.f}) {
    return WrappingMethod::horizontal;
  }
  return WrappingMethod::none;
}

const float NaN = numeric_limits<float>::quiet_NaN();

constexpr unsigned maxlevel(unsigned bits) { return (1u << bits) - 1u; }

template <unsigned bits> float expandValue(uint16_t x) {
  return float(x) / float(maxlevel(bits));
}

template <unsigned bits> uint16_t quantizeValue(float x) {
  if (x > 0) {
    unsigned y = unsigned(float(maxlevel(bits)) * x);
    return static_cast<uint16_t>(min(y, maxlevel(bits)));
  }
  return 0;
}

Mat3f expandTexture(const CameraParameters &camera, const TextureFrame &inYuv) {
  static_assert(is_same_v<TextureFrame, Frame<YUV420P10>>);

  auto &Y = inYuv.getPlane(0);
  auto &U = inYuv.getPlane(1);
  auto &V = inYuv.getPlane(2);
  Mat3f out(inYuv.getPlane(0).sizes());
  const auto width = Y.width();
  const auto height = Y.height();

  for (unsigned i = 0; i != height; ++i) {
    for (unsigned j = 0; j != width; ++j) {
      out(i, j) =
          Vec3f{expandValue<10u>(Y(i, j)), expandValue<10u>(U(i / 2, j / 2)),
                expandValue<10u>(V(i / 2, j / 2))};
    }
  }
  return out;
}

TextureFrame quantizeTexture(const CameraParameters &camera, const Mat3f &in) {
  static_assert(is_same_v<TextureFrame, Frame<YUV420P10>>);

  TextureFrame outYuv(in.width(), in.height());
  auto &Y = outYuv.getPlane(0);
  auto &U = outYuv.getPlane(1);
  auto &V = outYuv.getPlane(2);
  const auto width = Y.width();
  const auto height = Y.height();

  for (unsigned i = 0; i != height; ++i) {
    for (unsigned j = 0; j != width; ++j) {
      Y(i, j) = quantizeValue<10u>(in(i, j).x());
    }
  }
  for (unsigned i = 0; i != height / 2; ++i) {
    for (unsigned j = 0; j != width / 2; ++j) {
      U(i, j) = quantizeValue<10u>(
          0.25f * (in(2 * i, 2 * j).y() + in(2 * i, 2 * j + 1).y() +
                   in(2 * i + 1, 2 * j).y() + in(2 * i + 1, 2 * j + 1).y()));
      V(i, j) = quantizeValue<10u>(
          0.25f * (in(2 * i, 2 * j).z() + in(2 * i, 2 * j + 1).z() +
                   in(2 * i + 1, 2 * j).z() + in(2 * i + 1, 2 * j + 1).z()));
    }
  }
  return outYuv;
}

Mat1f expandDepth(const CameraParameters &camera, const DepthFrame &inYuv) {
  static_assert(is_same_v<DepthFrame, Frame<YUV400P16>> ||
                is_same_v<DepthFrame, Frame<YUV420P16>>);
  auto &in = inYuv.getPlane(0);
  Mat1f out(in.sizes());
  transform(begin(in), end(in), begin(out),
            [near = camera.depthRange[0],
             far = camera.depthRange[1]](uint16_t x) -> float {
              if (x > 0) {
                const float normDisp = expandValue<16u>(x);
                if (far >= 1000.f /*meter*/) {
                  return near / normDisp;
                }
                return far * near / (near + normDisp * (far - near));
              }
              return NaN;
            });
  return out;
}

DepthFrame quantizeNormDisp(const CameraParameters &camera, const Mat1f &in) {
  static_assert(is_same_v<DepthFrame, Frame<YUV400P16>> ||
                is_same_v<DepthFrame, Frame<YUV420P16>>);
  DepthFrame outYuv(in.width(), in.height());
  auto &out = outYuv.getPlane(0);
  transform(begin(in), end(in), begin(out),
            [near = camera.depthRange[0],
             far = camera.depthRange[1]](float normDisp) -> uint16_t {
              if (normDisp > 0. && isfinite(normDisp)) {
                if (far >= 1000.f /*meter*/) {
                  return quantizeValue<16u>(near * normDisp);
                }
                return quantizeValue<16u>((far * near * normDisp - near) /
                                          (far - near));
              }
              return 0;
            });
  return outYuv;
}
} // namespace

Synthesizer::Synthesizer(double rayAngleParam, double depthParam,
                         double stretchingParam)
    : m_rayAngleParam{rayAngleParam}, m_depthParam{depthParam},
      m_stretchingParam{stretchingParam} {}

TextureFrame Synthesizer::renderTexture(const MVDFrame &frame,
                                        const CameraParameterList &cameras,
                                        const CameraParameters &target) const {
  return renderTextureDepth(frame, cameras, target).first;
}

DepthFrame Synthesizer::renderDepth(const MVDFrame &frame,
                                    const CameraParameterList &cameras,
                                    const CameraParameters &target) const {
  return renderTextureDepth(frame, cameras, target).second;
}

TextureDepthFrame
Synthesizer::renderTextureDepth(const MVDFrame &frame,
                                const CameraParameterList &cameras,
                                const CameraParameters &target) const {
  AccumulatingView av{m_rayAngleParam, m_depthParam, m_stretchingParam};

  assert(frame.size() == cameras.size());
  auto i_camera = begin(cameras);

  for (const auto &view : frame) {
    auto &camera = *i_camera++;

    auto outPoints =
        changeReferenceFrame(camera, target,
                             unprojectPoints(camera, imagePositions(camera),
                                             expandDepth(camera, view.second)));
    auto positions_depth = projectPoints(target, outPoints);
    auto rayAngles = calculateRayAngles(camera, target, outPoints);

    av.transform(expandTexture(camera, view.first), positions_depth.first,
                 positions_depth.second, rayAngles, target.size,
                 wrappingMethod(target));
  }

  return {quantizeTexture(target, av.texture()),
          quantizeNormDisp(target, av.normDisp())};
}

Mat1f Synthesizer::renderDepth(const Mat1f &frame,
                               const CameraParameters &camera,
                               const CameraParameters &target) const {
  AccumulatingView av{m_rayAngleParam, m_depthParam, m_stretchingParam};

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
