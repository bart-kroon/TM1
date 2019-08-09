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

#ifndef _TMIV_IMAGE_IMAGE_H_
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/Common.h>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

namespace TMIV::Image {
constexpr auto kilometer = 1000.F;

constexpr unsigned maxLevel(unsigned bits) { return (1U << bits) - 1U; }

template <unsigned bits> float expandValue(uint16_t x) {
  return float(x) / float(maxLevel(bits));
}

template <unsigned bits> uint16_t quantizeValue(float x) {
  if (x >= 0.F && x <= 1.F) {
    return static_cast<uint16_t>(std::min(
        unsigned(std::lround(x * float(maxLevel(bits)))), maxLevel(bits)));
  }
  if (x > 0) {
    return static_cast<uint16_t>(maxLevel(bits));
  }
  return 0;
}

template <typename ToInt, typename WorkInt>
auto requantizeValue(WorkInt x, WorkInt fromBits, WorkInt toBits) -> ToInt {
  static_assert(std::is_integral_v<WorkInt> && std::is_unsigned_v<WorkInt>);
  static_assert(std::is_integral_v<ToInt> && std::is_unsigned_v<ToInt>);
  assert(0 < fromBits && 0 < toBits &&
         toBits <= std::numeric_limits<ToInt>::digits &&
         toBits + fromBits <= std::numeric_limits<WorkInt>::digits);

  const auto maxFrom = (1U << fromBits) - 1U;
  const auto maxTo = (1U << toBits) - 1U;

  assert(0U <= x && x <= maxFrom);

  return ToInt((x * maxTo + maxFrom / 2U) / maxFrom);
}

template <typename OutFormat, typename InFormat>
auto requantize(const Common::Frame<InFormat> &frame, unsigned bits)
    -> Common::Frame<OutFormat> {
  using InTraits = Common::detail::PixelFormatHelper<InFormat>;
  using OutTraits = Common::detail::PixelFormatHelper<OutFormat>;
  using OutInt = typename OutTraits::base_type;
  using WorkInt = uint_fast32_t;
  constexpr auto outBits = OutTraits::bitDepth;
  constexpr auto numPlanes = std::min(InTraits::nb_plane, OutTraits::nb_plane);

  auto result = Common::Frame<OutFormat>(frame.getWidth(), frame.getHeight());
  for (int i = 0; i < numPlanes; ++i) {
    assert(frame.getPlane(i).width() == result.getPlane(i).width());
    assert(frame.getPlane(i).height() == result.getPlane(i).height());
    std::transform(std::begin(frame.getPlane(i)), std::end(frame.getPlane(i)),
                   std::begin(result.getPlane(i)), [=](WorkInt x) {
                     return requantizeValue<OutInt, WorkInt>(x, bits, outBits);
                   });
  }
  return result;
}

template <typename OutFormat, typename InFormat>
auto requantize(const Common::MVDFrame<InFormat> &frame, unsigned bits)
    -> Common::MVDFrame<OutFormat> {
  auto result = Common::MVDFrame<OutFormat>();
  result.reserve(frame.size());
  std::transform(
      std::begin(frame), std::end(frame), std::back_inserter(result),
      [=](const Common::TextureDepthFrame<InFormat> &view)
          -> Common::TextureDepthFrame<OutFormat> {
        return {view.first, requantize<OutFormat>(view.second, bits)};
      });
  return result;
}

template <unsigned bits>
float expandDepthValue(const Metadata::CameraParameters &camera, uint16_t x) {
  const auto near = camera.depthRange[0];
  const auto far = camera.depthRange[1];

  const float normDisp = expandValue<bits>(x);
  if (far >= kilometer) {
    return near / normDisp;
  }
  return far * near / (near + normDisp * (far - near));
}
} // namespace TMIV::Image
