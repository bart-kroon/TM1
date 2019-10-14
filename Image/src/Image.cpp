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

#include <TMIV/Common/Common.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

using Mat1f = Mat<float>;
using Mat3f = Mat<Vec3f>;

namespace TMIV::Image {
auto modifyDepthRange(const MVD16Frame &frame16, const CameraParametersVector &cameras16,
                      const CameraParametersVector &cameras10) -> MVD10Frame {
  auto frame10 = MVD10Frame{};
  frame10.reserve(frame16.size());
  auto i_camera16 = begin(cameras16);
  auto i_camera10 = begin(cameras10);
  transform(begin(frame16), end(frame16), back_inserter(frame10),
            [&](const TextureDepth16Frame &view16) -> TextureDepth10Frame {
              auto view10 = TextureDepth10Frame{
                  view16.first, Depth10Frame{view16.second.getWidth(), view16.second.getHeight()}};
              transform(begin(view16.second.getPlane(0)), end(view16.second.getPlane(0)),
                        begin(view10.second.getPlane(0)), [&](uint16_t x) {
                          const auto normDisp = impl::expandNormDispValue<16>(*i_camera16, x);
                          return impl::quantizeNormDispValue<10>(*i_camera10, normDisp);
                        });
              ++i_camera16;
              ++i_camera10;
              return view10;
            });
  return frame10;
}

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
      out(i, j) =
          Vec3f{impl::expandValue<bitDepth>(Y(i, j)), impl::expandValue<bitDepth>(U(i / 2, j / 2)),
                impl::expandValue<bitDepth>(V(i / 2, j / 2))};
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
        outYuv.getPlane(k)(i, j) = impl::quantizeValue<bitDepth>(in(i, j)[k]);
      }
    }
  }

  return outYuv;
}

Mat<float> expandDepth(const CameraParameters &camera, const Depth10Frame &in) {
  auto out = Mat<float>({size_t(in.getHeight()), size_t(in.getWidth())});
  transform(begin(in.getPlane(0)), end(in.getPlane(0)), begin(out),
            [&](uint16_t x) { return impl::expandDepthValue<10>(camera, x); });
  return out;
}

Mat<float> expandDepth(const CameraParameters &camera, const Depth16Frame &in) {
  auto out = Mat<float>({size_t(in.getHeight()), size_t(in.getWidth())});
  transform(begin(in.getPlane(0)), end(in.getPlane(0)), begin(out),
            [&](uint16_t x) { return impl::expandDepthValue<16>(camera, x); });
  return out;
}

Depth16Frame quantizeNormDisp16(const CameraParameters &camera, const Mat1f &in) {
  auto out = Depth16Frame{int(in.width()), int(in.height())};
  transform(begin(in), end(in), begin(out.getPlane(0)),
            [&](float x) { return impl::quantizeNormDispValue<16>(camera, x); });
  return out;
}
} // namespace TMIV::Image
