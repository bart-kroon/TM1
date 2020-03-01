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

#include <TMIV/Common/Frame.h>

#include <TMIV/Common/Common.h>

#include <cassert>

using namespace std;

namespace TMIV::Common {
namespace {
template <class TO, class FROM> auto yuv420p_impl(const Frame<FROM> &frame) -> Frame<TO> {
  Frame<TO> result(frame.getWidth(), frame.getHeight());
  copy(begin(frame.getPlane(0)), end(frame.getPlane(0)), begin(result.getPlane(0)));

  assert(frame.getWidth() % 2 == 0 && frame.getHeight() % 2 == 0);
  const int rows = result.getHeight() / 2;
  const int cols = result.getWidth() / 2;

  for (int k = 1; k < 3; ++k) {
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        auto sum = frame.getPlane(k)(2 * i, 2 * j) + frame.getPlane(k)(2 * i + 1, 2 * j) +
                   frame.getPlane(k)(2 * i, 2 * j + 1) + frame.getPlane(k)(2 * i + 1, 2 * j + 1);
        result.getPlane(k)(i, j) = (sum + 2) / 4;
      }
    }
  }

  return result;
}

template <class TO, class FROM> auto yuv444p_impl(const Frame<FROM> &frame) -> Frame<TO> {
  assert(frame.getWidth() % 2 == 0 && frame.getHeight() % 2 == 0);

  auto result = Frame<TO>{frame.getWidth(), frame.getHeight()};

  for (int i = 0; i < frame.getHeight(); ++i) {
    for (int j = 0; j < frame.getWidth(); ++j) {
      result.getPlane(0)(i, j) = frame.getPlane(0)(i, j);
      result.getPlane(0)(i, j) = frame.getPlane(0)(i / 2, j / 2);
      result.getPlane(0)(i, j) = frame.getPlane(0)(i / 2, j / 2);
    }
  }

  return result;
}
} // namespace

auto yuv420p(const Frame<YUV444P8> &frame) -> Frame<YUV420P8> {
  return yuv420p_impl<YUV420P8>(frame);
}

auto yuv420p(const Frame<YUV444P10> &frame) -> Frame<YUV420P10> {
  return yuv420p_impl<YUV420P10>(frame);
}

auto yuv420p(const Frame<YUV444P16> &frame) -> Frame<YUV420P16> {
  return yuv420p_impl<YUV420P16>(frame);
}

auto yuv444p(const Frame<YUV420P8> &frame) -> Frame<YUV444P8> {
  return yuv444p_impl<YUV444P8>(frame);
}

auto yuv444p(const Frame<YUV420P10> &frame) -> Frame<YUV444P10> {
  return yuv444p_impl<YUV444P10>(frame);
}

auto yuv444p(const Frame<YUV420P16> &frame) -> Frame<YUV444P16> {
  return yuv444p_impl<YUV444P16>(frame);
}

auto expandTexture(const Frame<YUV444P10> &inYuv) -> Mat<Vec3f> {
  auto &Y = inYuv.getPlane(0);
  auto &U = inYuv.getPlane(1);
  auto &V = inYuv.getPlane(2);
  Mat<Vec3f> out(inYuv.getPlane(0).sizes());
  const auto width = Y.width();
  const auto height = Y.height();
  constexpr auto bitDepth = 10U;

  for (unsigned i = 0; i != height; ++i) {
    for (unsigned j = 0; j != width; ++j) {
      out(i, j) = Vec3f{expandValue<bitDepth>(Y(i, j)), expandValue<bitDepth>(U(i, j)),
                        expandValue<bitDepth>(V(i, j))};
    }
  }
  return out;
}

auto quantizeTexture(const Mat<Vec3f> &in) -> Frame<YUV444P10> {
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
} // namespace TMIV::Common
