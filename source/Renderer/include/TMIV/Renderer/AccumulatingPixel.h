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

#ifndef TMIV_RENDERER_ACCUMULATINGPIXEL_H
#define TMIV_RENDERER_ACCUMULATINGPIXEL_H

#include <TMIV/Common/LinAlg.h>

namespace TMIV::Renderer {
// The information that is kept per pixel to blend multiple pixels
//
// With empty base class initialization
struct PixelAccumulator {
  PixelAccumulator() = default;
  PixelAccumulator(const PixelAccumulator &) = default;
  PixelAccumulator(PixelAccumulator &&) noexcept = default;
  auto operator=(const PixelAccumulator &) -> PixelAccumulator & = default;
  auto operator=(PixelAccumulator &&) noexcept -> PixelAccumulator & = default;
  ~PixelAccumulator() = default;

  PixelAccumulator(Common::Vec3f color_, float normWeight_, float normDisp_, float stretching_);

  Common::Vec3f color{};

  // weight is implicit as normWeight *
  // AccumulatingPixel<T>::normDispWeight(normDisp) but never directly
  // calculated to avoid numerical instability.
  float normWeight{0.F};

  // Normalized disparity in diopters
  float normDisp{0.F};

  // Stretching as a ratio of areas
  float stretching{0.F};

  // Depth in meters
  [[nodiscard]] constexpr auto depth() const -> float { return 1.F / normDisp; }
};

// The result of the blending process for a single pixel
//
// With empty base class initialization
struct PixelValue {
  PixelValue() = default;
  PixelValue(const PixelValue &) = default;
  PixelValue(PixelValue &&) noexcept = default;
  auto operator=(const PixelValue &) -> PixelValue & = default;
  auto operator=(PixelValue &&) noexcept -> PixelValue & = default;
  ~PixelValue() = default;

  PixelValue(Common::Vec3f color_, float normDisp_, float normWeight_, float stretching_);

  Common::Vec3f color{};

  // Normalized disparity in diopters
  float normDisp{0.F};

  // The normalized weight serves as a quality indication
  float normWeight{0.F};

  // Amount of stretching as a ratio of area (another quality indication)
  float stretching{0.F};

  // Depth in meters
  [[nodiscard]] constexpr auto depth() const -> float { return 1.F / normDisp; }
};

// Pixel blending operations
class AccumulatingPixel {
public:
  const float rayAngleParam;
  const float depthParam;
  const float stretchingParam;
  const float maxStretching;

  AccumulatingPixel(float rayAngleParam_, float depthParam_, float stretchingParam_,
                    float maxStretching_);

  // Construct a pixel accumulator from a single synthesized pixel
  [[nodiscard]] auto construct(Common::Vec3f color, float normDisp, float rayAngle,
                               float stretching) const -> PixelAccumulator;

private:
  // Blend two pixels with known blending weights
  [[nodiscard]] auto blendAccumulators(float w_a, const PixelAccumulator &a, float w_b,
                                       const PixelAccumulator &b) const -> PixelAccumulator;

public:
  // Blend two pixels
  [[nodiscard]] auto blend(const PixelAccumulator &a, const PixelAccumulator &b) const
      -> PixelAccumulator;

  // Average a pixel
  [[nodiscard]] auto average(PixelAccumulator const &x) const -> PixelValue;

  // Calculate the weight of a pixel based on cosine of the ray angle between input and virtual ray
  // only
  [[nodiscard]] auto rayAngleWeight(float rayAngle) const -> float;

  // Calculate the weight of a pixel based on normalized disparity (diopters) only
  [[nodiscard]] auto normDispWeight(float normDisp) const -> float;

  // Calculate the weight of a pixel based on stretching only
  [[nodiscard]] auto stretchingWeight(float stretching) const -> float;
};
} // namespace TMIV::Renderer

#endif
