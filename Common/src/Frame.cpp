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

#include <cassert>

namespace TMIV::Common {
namespace {
template <class TO, class FROM> auto yuv420p_impl(const Frame<FROM> &frame) -> Frame<TO> {
  Frame<TO> result(frame.getWidth(), frame.getHeight());
  std::copy(std::begin(frame.getPlane(0)), std::end(frame.getPlane(0)),
            std::begin(result.getPlane(0)));

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
} // namespace

Frame<YUV420P8> yuv420p(const Frame<YUV444P8> &frame) { return yuv420p_impl<YUV420P8>(frame); }

Frame<YUV420P10> yuv420p(const Frame<YUV444P10> &frame) { return yuv420p_impl<YUV420P10>(frame); }

Frame<YUV420P16> yuv420p(const Frame<YUV444P16> &frame) { return yuv420p_impl<YUV420P16>(frame); }
} // namespace TMIV::Common
