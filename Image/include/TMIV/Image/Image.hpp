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
#include <cstdint>
#include <limits>

namespace TMIV::Image {
constexpr unsigned maxLevel(unsigned bits) { return (1u << bits) - 1u; }

template <unsigned bits> float expandValue(uint16_t x) {
  return float(x) / float(maxLevel(bits));
}

template <unsigned bits> uint16_t quantizeValue(float x) {
  if (x >= 0.f && x <= 1.f) {
    return static_cast<uint16_t>(
        std::min(unsigned(0.5f + x * float(maxLevel(bits))), maxLevel(bits)));
  }
  if (x > 0) {
    return static_cast<uint16_t>(maxLevel(bits));
  }
  return 0u;
}

template <unsigned from_bits, unsigned to_bits>
uint16_t requantizeValue(uint16_t x) {
  return quantizeValue<to_bits>(expandValue<from_bits>(x));
}

template <unsigned bits>
float expandDepthValue(const Metadata::CameraParameters &camera, uint16_t x) {
  const auto near = camera.depthRange[0];
  const auto far = camera.depthRange[1];

  if (x > 0) {
    const float normDisp = expandValue<bits>(x);
    if (far >= 1000.f /*meter*/) {
      return near / normDisp;
    }
    return far * near / (near + normDisp * (far - near));
  }
  return Common::NaN;
}
} // namespace TMIV::Image
