/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include <TMIV/Renderer/AccumulatingPixel.h>

#include <cmath>

namespace TMIV::Renderer {
namespace {
// Blend two arithmetic tensors of fixed size
template <typename T> auto blendValues(float w_a, T a, float w_b, T b) -> T {
  if constexpr (std::is_floating_point_v<T>) {
    return w_a * a + w_b * b;
  } else if constexpr (std::is_integral_v<T>) {
    return static_cast<T>(std::lround(w_a * static_cast<float>(a) + w_b * static_cast<float>(b)));
  } else {
    T result;
    static_assert(result.size() == a.size()); // req. constexpr size()
    for (uint32_t i = 0; i < result.size(); ++i) {
      result[i] = blendValues(w_a, a[i], w_b, b[i]);
    }
    return result;
  }
}
} // namespace

PixelAccumulator::PixelAccumulator(Common::Vec3f color_, float normWeight_, float normDisp_,
                                   float stretching_)
    : color{color_}, normWeight{normWeight_}, normDisp{normDisp_}, stretching{stretching_} {}

PixelValue::PixelValue(Common::Vec3f color_, float normDisp_, float normWeight_, float stretching_)
    : color{color_}, normDisp{normDisp_}, normWeight{normWeight_}, stretching{stretching_} {}

AccumulatingPixel::AccumulatingPixel(float rayAngleParam_, float depthParam_,
                                     float stretchingParam_, float maxStretching_)
    : rayAngleParam{rayAngleParam_}
    , depthParam{depthParam_}
    , stretchingParam{stretchingParam_}
    , maxStretching{maxStretching_} {}

auto AccumulatingPixel::construct(Common::Vec3f color, float normDisp, float rayAngle,
                                  float stretching) const -> PixelAccumulator {
  return {color, rayAngleWeight(rayAngle) * stretchingWeight(stretching), normDisp, stretching};
}

auto AccumulatingPixel::blendAccumulators(float w_a, const PixelAccumulator &a, float w_b,
                                          const PixelAccumulator &b) const -> PixelAccumulator {
  const float normDisp = blendValues(w_a, a.normDisp, w_b, b.normDisp);
  const float normWeight = a.normWeight * normDispWeight(a.normDisp - normDisp) +
                           b.normWeight * normDispWeight(b.normDisp - normDisp);
  return PixelAccumulator{blendValues(w_a, a.color, w_b, b.color), normWeight, normDisp,
                          blendValues(w_a, a.stretching, w_b, b.stretching)};
}

auto AccumulatingPixel::blend(const PixelAccumulator &a, const PixelAccumulator &b) const
    -> PixelAccumulator {
  // Trivial blends occur often for atlases
  if (!(a.normWeight > 0.F)) {
    return b;
  }
  if (!(b.normWeight > 0.F)) {
    return a;
  }

  // Normalize weights on the nearest pixel
  if (a.normDisp >= b.normDisp) {
    // a is in front of b
    const float w_a =
        a.normWeight / (a.normWeight + b.normWeight * normDispWeight(b.normDisp - a.normDisp));
    const float w_b = 1.F - w_a;

    // Optimization: No alpha blending when w_b is almost zero
    if (w_b < 0.01F) {
      return a;
    }

    // Full alpha blend
    return blendAccumulators(w_a, a, w_b, b);
  }
  // b is in front of a
  const float w_b =
      b.normWeight / (b.normWeight + a.normWeight * normDispWeight(a.normDisp - b.normDisp));
  const float w_a = 1.F - w_b;

  // Optimization: No alpha blending when w_a is almost zero
  if (w_a < 0.01F) {
    return b;
  }

  // Full alpha blend
  return blendAccumulators(w_a, a, w_b, b);
}

auto AccumulatingPixel::average(PixelAccumulator const &x) const -> PixelValue {
  if (x.normWeight > 0.F && x.stretching < maxStretching) {
    return {x.color, x.normDisp, x.normWeight, x.stretching};
  }
  return {{}, 0.F, 0.F, 0.F};
}

auto AccumulatingPixel::rayAngleWeight(float rayAngle) const -> float {
  return std::exp(-rayAngleParam * rayAngle);
}

auto AccumulatingPixel::normDispWeight(float normDisp) const -> float {
  return std::exp(depthParam * normDisp);
}

auto AccumulatingPixel::stretchingWeight(float stretching) const -> float {
  return std::exp(-stretchingParam * stretching);
}
} // namespace TMIV::Renderer
