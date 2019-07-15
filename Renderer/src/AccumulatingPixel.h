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

#ifndef _TMIV_RENDERER_ACCUMULATINGPIXEL_H_
#define _TMIV_RENDERER_ACCUMULATINGPIXEL_H_

#include "blend.h"
#include <TMIV/Common/LinAlg.h>
#include <cassert>
#include <cmath>

namespace TMIV::Renderer {
// The attributes that are blended
template <typename... T> using PixelAttributes = std::tuple<T...>;

// The information that is kept per pixel to blend multiple pixels
//
// With empty base class initialization
template <typename... T>
struct PixelAccumulator : private PixelAttributes<T...> {
  PixelAccumulator() = default;
  PixelAccumulator(const PixelAccumulator &) = default;
  PixelAccumulator(PixelAccumulator &&) = default;
  PixelAccumulator &operator=(const PixelAccumulator &) = default;
  PixelAccumulator &operator=(PixelAccumulator &&) = default;
  ~PixelAccumulator() = default;

  PixelAccumulator(PixelAttributes<T...> attributes, float normWeight_,
                   float normDisp_, float stretching_)
      : PixelAttributes<T...>{attributes},
        normWeight{normWeight_}, normDisp{normDisp_}, stretching{stretching_} {
    assert(normWeight_ >= 0.f);
    assert(normDisp_ >= 0.f);
    assert(stretching_ > 0.f);
  }

  // weight is implicit as normWeight *
  // AccumulatingPixel<T...>::normDispWeight(normDisp) but never directly
  // calculated to avoid numerical instability.
  float normWeight{0.f};

  // Normalized disparity in diopters
  float normDisp{0.f};

  // Stretching as a ratio of areas
  float stretching{0.f};

  // Access the attributes
  const PixelAttributes<T...> &attributes() const { return *this; }

  // Access the attributes
  PixelAttributes<T...> &attributes() { return *this; }

  // Depth in meters
  constexpr float depth() const { return 1.f / normDisp; }
};

// The result of the blending process for a single pixel
//
// With empty base class initialization
template <typename... T> struct PixelValue : private PixelAttributes<T...> {
  PixelValue() = default;
  PixelValue(const PixelValue &) = default;
  PixelValue(PixelValue &&) = default;
  PixelValue &operator=(const PixelValue &) = default;
  PixelValue &operator=(PixelValue &&) = default;

  PixelValue(PixelAttributes<T...> attributes, float normDisp_,
             float normWeight_, float stretching_)
      : PixelAttributes<T...>{attributes}, normDisp{normDisp_},
        normWeight{normWeight_}, stretching{stretching_} {
    assert(normDisp_ >= 0.f);
    assert(normWeight_ >= 0.f);
    assert(stretching_ >= 0.f);
  }

  // Normalized disparity in diopters
  float normDisp{0.f};

  // The normalized weight serves as a quality indication
  float normWeight{0.f};

  // Amount of stretching as a ratio of area (another quality indication)
  float stretching{0.f};

  // Access the attributes
  const PixelAttributes<T...> &attributes() const { return *this; }

  // Access the attributes
  PixelAttributes<T...> &attributes() { return *this; }

  // Depth in meters
  constexpr float depth() const { return 1.f / normDisp; }
};

// Pixel blending operations
template <typename... T> class AccumulatingPixel {
private:
  using Accumulator = PixelAccumulator<T...>;
  using Attributes = PixelAttributes<T...>;
  using Value = PixelValue<T...>;

public:
  const float rayAngleParam;
  const float depthParam;
  const float stretchingParam;
  const float maxStretching;

  AccumulatingPixel(float rayAngleParam_, float depthParam_,
                    float stretchingParam_, float maxStretching_)
      : rayAngleParam{rayAngleParam_}, depthParam{depthParam_},
        stretchingParam{stretchingParam_}, maxStretching{maxStretching_} {}

  // Construct a pixel accumulator from a single synthesized pixel
  auto construct(Attributes attributes, float normDisp, float rayAngle,
                 float stretching) const -> Accumulator {
    assert(normDisp >= 0.f);
    return {attributes, rayAngleWeight(rayAngle) * stretchingWeight(stretching),
            normDisp, stretching};
  }

private:
  // Blend two pixels with known blending weights
  auto blendAccumulators(float w_a, const Accumulator &a, float w_b,
                         const Accumulator &b) const -> Accumulator {
    const float normDisp = blendValues(w_a, a.normDisp, w_b, b.normDisp);
    const float normWeight =
        a.normWeight * normDispWeight(a.normDisp - normDisp) +
        b.normWeight * normDispWeight(b.normDisp - normDisp);
    return Accumulator{
        blendAttributes(w_a, a.attributes(), w_b, b.attributes()), normWeight,
        normDisp, blendValues(w_a, a.stretching, w_b, b.stretching)};
  }

public:
  // Blend two pixels
  auto blend(const Accumulator &a, const Accumulator &b) const -> Accumulator {
    // Trivial blends occur often for atlases
    if (!(a.normWeight > 0.f)) {
      return b;
    }
    if (!(b.normWeight > 0.f)) {
      return a;
    }

    // Normalize weights on the nearest pixel
    if (a.normDisp >= b.normDisp) {
      // a is in front of b
      const float w_a =
          a.normWeight /
          (a.normWeight +
           b.normWeight * normDispWeight(b.normDisp - a.normDisp));
      assert(w_a >= 0.f);
      const float w_b = 1.f - w_a;

      // Optimization: No alpha blending when w_b is almost zero
      if (w_b < 0.01f) {
        return a;
      }

      // Full alpha blend
      return blendAccumulators(w_a, a, w_b, b);
    } else {
      // b is in front of a
      const float w_b =
          b.normWeight /
          (b.normWeight +
           a.normWeight * normDispWeight(a.normDisp - b.normDisp));
      assert(w_b >= 0.f);
      const float w_a = 1.f - w_b;

      // Optimization: No alpha blending when w_a is almost zero
      if (w_a < 0.01f) {
        return b;
      }

      // Full alpha blend
      return blendAccumulators(w_a, a, w_b, b);
    }
  }

  // Average a pixel
  Value average(Accumulator const &x) const {
    if (x.normWeight > 0.f && x.stretching < maxStretching) {
      return {x.attributes(), x.normDisp, x.normWeight, x.stretching};
    }
    return {Attributes{}, 0.f, 0.f, 0.f};
  }

  // Calculate the weight of a pixel based on cosine of the ray
  // angle between input and virtual ray only
  float rayAngleWeight(float rayAngle) const {
    return std::exp(-rayAngleParam * rayAngle);
  }

  // Calculate the weight of a pixel based on normalized disparity
  // (diopters) only
  float normDispWeight(float normDisp) const {
    return std::exp(depthParam * normDisp);
  }

  // Calculate the weight of a pixel based on stretching only
  float stretchingWeight(float stretching) const {
    return std::exp(-stretchingParam * stretching);
  }
};
} // namespace TMIV::Renderer

#endif
