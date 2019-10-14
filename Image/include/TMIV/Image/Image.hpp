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

namespace TMIV::Image {
constexpr unsigned maxLevel(unsigned bits) { return (1U << bits) - 1U; }

namespace impl {
// An implementation-specific maximum depth value
constexpr auto kilometer = 1000.F;

template <unsigned bits> float expandValue(uint16_t x) { return float(x) / float(maxLevel(bits)); }

template <unsigned bits> uint16_t quantizeValue(float x) {
  if (x >= 0.F && x <= 1.F) {
    return static_cast<uint16_t>(
        std::min(unsigned(std::lround(x * float(maxLevel(bits)))), maxLevel(bits)));
  }
  if (x > 0) {
    return static_cast<uint16_t>(maxLevel(bits));
  }
  return 0;
}

template <unsigned bits>
float expandNormDispValue(const Metadata::ViewParams &viewParams, uint16_t x) {
  if (x >= viewParams.depthOccMapThreshold) {
    const auto &R = viewParams.normDispRange;
    return std::max(1.F / kilometer, R[0] + (R[1] - R[0]) * expandValue<bits>(x));
  }
  return 0.F;
}

template <unsigned bits>
float expandDepthValue(const Metadata::ViewParams &viewParams, uint16_t x) {
  const auto normDisp = expandNormDispValue<bits>(viewParams, x);
  return normDisp > 0.F ? 1.F / normDisp : 0.F;
}

template <unsigned bits>
uint16_t quantizeNormDispValue(const Metadata::ViewParams &viewParams, float x) {
  if (x > 0.F && std::isfinite(x)) {
    const auto &R = viewParams.normDispRange;
    const auto value = quantizeValue<bits>((x - R[0]) / (R[1] - R[0]));
    return std::max(viewParams.depthOccMapThreshold, value);
  }
  return 0;
}

template <unsigned bits>
uint16_t quantizeDepthValue(const Metadata::ViewParams &viewParams, float x) {
  return x > 0.F ? quantizeNormDispValue<bits>(camera, 1.F / x) : 0;
}
} // namespace impl

inline float expandDepthValue10(const Metadata::ViewParams &viewParams, uint16_t x) {
  return impl::expandDepthValue<10>(viewParams, x);
}

inline float expandDepthValue16(const Metadata::ViewParams &viewParams, uint16_t x) {
  return impl::expandDepthValue<16>(viewParams, x);
}
} // namespace TMIV::Image
