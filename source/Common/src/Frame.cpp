/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/Common.h>
#include <TMIV/Common/Thread.h>

#include <sstream>

namespace TMIV::Common {
auto expandTexture(const Frame<> &inYuv) -> Mat<Vec3f> {
  const auto &Y = inYuv.getPlane(0);
  const auto &U = inYuv.getPlane(1);
  const auto &V = inYuv.getPlane(2);
  Mat<Vec3f> out(inYuv.getPlane(0).sizes());
  const auto width = Y.width();
  const auto height = Y.height();
  const auto bitDepth = inYuv.getBitDepth();

  for (uint32_t i = 0; i != height; ++i) {
    for (uint32_t j = 0; j != width; ++j) {
      out(i, j) = Vec3f{expandValue(Y(i, j), bitDepth), expandValue(U(i, j), bitDepth),
                        expandValue(V(i, j), bitDepth)};
    }
  }
  return out;
}

auto expandLuma(const Frame<> &inYuv) -> Mat<float> {
  auto out = Mat<float>(inYuv.getPlane(0).sizes());
  std::transform(
      inYuv.getPlane(0).cbegin(), inYuv.getPlane(0).cend(), out.begin(),
      [bitDepth = inYuv.getBitDepth()](auto value) { return expandValue(value, bitDepth); });
  return out;
}

auto quantizeTexture(const Mat<Vec3f> &in, uint32_t bitDepth) -> Frame<> {
  auto outYuv = Frame<>{Vec2i{static_cast<int32_t>(in.width()), static_cast<int32_t>(in.height())},
                        bitDepth, ColorFormat::YUV444};
  const auto width = in.width();
  const auto height = in.height();

  for (int32_t k = 0; k < 3; ++k) {
    for (uint32_t i = 0; i != height; ++i) {
      for (uint32_t j = 0; j != width; ++j) {
        outYuv.getPlane(k)(i, j) = quantizeValue<uint16_t>(in(i, j)[k], bitDepth);
      }
    }
  }

  return outYuv;
}
} // namespace TMIV::Common
