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

#include <TMIV/Image/Image.h>
#include <cmath>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

using Mat1f = Mat<float>;
using Mat3f = Mat<Vec3f>;

namespace TMIV::Image {
Mat3f expandTexture(const Frame<YUV420P10> &inYuv) {
  auto &Y = inYuv.getPlane(0);
  auto &U = inYuv.getPlane(1);
  auto &V = inYuv.getPlane(2);
  Mat3f out(inYuv.getPlane(0).sizes());
  const auto width = Y.width();
  const auto height = Y.height();
  constexpr auto bitDepth = 10U;

  for (unsigned i = 0; i != height; ++i) {
    for (unsigned j = 0; j != width; ++j) {
      out(i, j) = Vec3f{expandValue<bitDepth>(Y(i, j)), expandValue<bitDepth>(U(i / 2, j / 2)),
                        expandValue<bitDepth>(V(i / 2, j / 2))};
    }
  }
  return out;
}

Frame<YUV444P10> quantizeTexture(const Mat3f &in) {
  Frame<YUV444P10> outYuv(int(in.width()), int(in.height()));
  const auto width = in.width();
  const auto height = in.height();

  for (int k = 0; k < 3; ++k) {
    for (unsigned i = 0; i != height; ++i) {
      for (unsigned j = 0; j != width; ++j) {
        constexpr auto bitDepth = 10U;
        outYuv.getPlane(k)(i, j) = quantizeValue<bitDepth>(in(i, j)[k]);
      }
    }
  }

  return outYuv;
}

namespace {
template <unsigned bits, class FRAME>
Mat1f expandDepth_impl(const CameraParameters &camera, const FRAME &inYuv) {
  auto &in = inYuv.getPlane(0);
  Mat1f out(in.sizes());
  transform(begin(in), end(in), begin(out),
            [&camera](uint16_t x) { return expandDepthValue<bits>(camera, x); });
  return out;
}
} // namespace

Mat1f expandDepth(const CameraParameters &camera, const Frame<YUV400P10> &inYuv) {
  constexpr auto bitDepth = 10;
  return expandDepth_impl<bitDepth>(camera, inYuv);
}

Mat1f expandDepth(const CameraParameters &camera, const Frame<YUV400P16> &inYuv) {
  constexpr auto bitDepth = 16;
  return expandDepth_impl<bitDepth>(camera, inYuv);
}

namespace {
template <unsigned bits, class FRAME>
FRAME quantizeNormDisp_impl(const CameraParameters &camera, const Mat1f &in) {
  FRAME outYuv(int(in.width()), int(in.height()));
  auto &out = outYuv.getPlane(0);
  transform(begin(in), end(in), begin(out),
            [near = camera.depthRange[0], far = camera.depthRange[1]](float normDisp) -> uint16_t {
              if (normDisp > 0.F && isfinite(normDisp)) {
                if (far >= kilometer) {
                  auto value = quantizeValue<bits>(near * normDisp);
                  return value > 0U ? value : 1U;
                }
                auto value = quantizeValue<bits>((far * near * normDisp - near) / (far - near));
                return value > 0U ? value : 1U;
              }
              return 0U;
            });
  return outYuv;
}

template <unsigned bits, class FRAME>
FRAME quantizeDepth_impl(const CameraParameters &camera, const Mat1f &in) {
  FRAME outYuv(int(in.width()), int(in.height()));
  auto &out = outYuv.getPlane(0);
  transform(begin(in), end(in), begin(out),
            [near = camera.depthRange[0], far = camera.depthRange[1]](float normDisp) -> uint16_t {
              if (normDisp > 0.F && isfinite(normDisp)) {
                if (far >= kilometer) {
                  return quantizeValue<bits>(near / normDisp);
                }
                return quantizeValue<bits>((far * near / normDisp - near) / (far - near));
              }
              return 0;
            });
  return outYuv;
}
} // namespace

Frame<YUV400P10> quantizeNormDisp10(const CameraParameters &camera, const Mat1f &in) {
  constexpr int bitDepth = 10;
  return quantizeNormDisp_impl<bitDepth, Frame<YUV400P10>>(camera, in);
}

Frame<YUV400P16> quantizeNormDisp16(const CameraParameters &camera, const Mat1f &in) {
  constexpr int bitDepth = 16;
  return quantizeNormDisp_impl<bitDepth, Frame<YUV400P16>>(camera, in);
}

Frame<YUV400P10> quantizeDepth10(const CameraParameters &camera, const Mat1f &in) {
  constexpr int bitDepth = 10;
  return quantizeDepth_impl<bitDepth, Frame<YUV400P10>>(camera, in);
}

Frame<YUV400P16> quantizeDepth16(const CameraParameters &camera, const Mat1f &in) {
  constexpr int bitDepth = 16;
  return quantizeDepth_impl<bitDepth, Frame<YUV400P16>>(camera, in);
}
} // namespace TMIV::Image
