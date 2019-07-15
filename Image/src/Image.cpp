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

  for (unsigned i = 0; i != height; ++i) {
    for (unsigned j = 0; j != width; ++j) {
      out(i, j) =
          Vec3f{expandValue<10u>(Y(i, j)), expandValue<10u>(U(i / 2, j / 2)),
                expandValue<10u>(V(i / 2, j / 2))};
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
        outYuv.getPlane(k)(i, j) = quantizeValue<10u>(in(i, j)[k]);
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
  transform(begin(in), end(in), begin(out), [&camera](uint16_t x) {
    return expandDepthValue<bits>(camera, x);
  });
  return out;
}
} // namespace

Mat1f expandDepth(const CameraParameters &camera,
                  const Frame<YUV400P10> &inYuv) {
  return expandDepth_impl<10>(camera, inYuv);
}

Mat1f expandDepth(const CameraParameters &camera,
                  const Frame<YUV400P16> &inYuv) {
  return expandDepth_impl<16>(camera, inYuv);
}

namespace {
template <unsigned bits, class FRAME>
FRAME quantizeNormDisp_impl(const CameraParameters &camera, const Mat1f &in) {
  FRAME outYuv(int(in.width()), int(in.height()));
  auto &out = outYuv.getPlane(0);
  transform(begin(in), end(in), begin(out),
            [near = camera.depthRange[0],
             far = camera.depthRange[1]](float normDisp) -> uint16_t {
              if (normDisp > 0.f && isfinite(normDisp)) {
                if (far >= 1000.f /*meter*/) {
                  return quantizeValue<bits>(near * normDisp);
                }
                return quantizeValue<bits>((far * near * normDisp - near) /
                                           (far - near));
              }
              return 0;
            });
  return outYuv;
}

template <unsigned bits, class FRAME>
FRAME quantizeDepth_impl(const CameraParameters &camera, const Mat1f &in) {
  FRAME outYuv(int(in.width()), int(in.height()));
  auto &out = outYuv.getPlane(0);
  transform(begin(in), end(in), begin(out),
            [near = camera.depthRange[0],
             far = camera.depthRange[1]](float normDisp) -> uint16_t {
              if (normDisp > 0.f && isfinite(normDisp)) {
                if (far >= 1000.f /*meter*/) {
                  return quantizeValue<bits>(near / normDisp);
                }
                return quantizeValue<bits>((far * near / normDisp - near) /
                                           (far - near));
              }
              return 0;
            });
  return outYuv;
}
} // namespace

Frame<YUV400P10> quantizeNormDisp10(const CameraParameters &camera,
                                    const Mat1f &in) {
  return quantizeNormDisp_impl<10, Frame<YUV400P10>>(camera, in);
}

Frame<YUV400P16> quantizeNormDisp16(const CameraParameters &camera,
                                    const Mat1f &in) {
  return quantizeNormDisp_impl<16, Frame<YUV400P16>>(camera, in);
}

Frame<YUV400P10> quantizeDepth10(const CameraParameters &camera,
                                 const Mat1f &in) {
  return quantizeDepth_impl<10, Frame<YUV400P10>>(camera, in);
}

Frame<YUV400P16> quantizeDepth16(const CameraParameters &camera,
                                 const Mat1f &in) {
  return quantizeDepth_impl<16, Frame<YUV400P16>>(camera, in);
}
} // namespace TMIV::Image
