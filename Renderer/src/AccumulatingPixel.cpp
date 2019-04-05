/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#include "AccumulatingPixel.h"

#include <cmath>

namespace TMIV::Renderer {
namespace {
float alphaBlend(float alpha, float x1, float x2) {
  return x1 + alpha * (x2 - x1);
}

using Common::Vec3f;
Vec3f alphaBlend(float alpha, const Vec3f &x1, const Vec3f &x2) {
  return Vec3f{alphaBlend(alpha, x1[0], x2[0]), alphaBlend(alpha, x1[1], x2[1]),
               alphaBlend(alpha, x1[2], x2[2])};
}
} // namespace

AccumulatingPixel::AccumulatingPixel(double rayAngleParam, double depthParam,
                                     double stretchingParam, Mode mode)
    : m_rayAngleParam(float(rayAngleParam)), m_depthParam(float(depthParam)),
      m_stretchingParam(float(stretchingParam)), m_mode(mode) {}

AccumulatingPixel::PixelAccumulator
AccumulatingPixel::construct(float pixelDepth, Vec3f pixelColor, float rayAngle,
                             float stretching) const {
  return {rayAngleWeight(rayAngle) * stretchingWeight(stretching), pixelColor,
          1.f / pixelDepth, rayAngle, stretching};
}

// Blend two pixels
AccumulatingPixel::PixelAccumulator
AccumulatingPixel::blend(AccumulatingPixel::PixelAccumulator const &a,
                         AccumulatingPixel::PixelAccumulator const &b) const {
  // NaN checks
  if (std::isnan(a.normDisp)) {
    return b;
  }
  if (std::isnan(b.normDisp)) {
    return a;
  }

  // Prefer small alpha and weight_ab for numerical stability
  if (a.normDisp > b.normDisp) {
    return blend(b, a);
  }
  // [a.normDisp - b.normDisp <= 0]

  auto weight_ab = normDispWeight(a.normDisp - b.normDisp);
  // [weight_ab <= 1]

  auto alpha = 1.f - b.normWeight / (weight_ab * a.normWeight + b.normWeight);
  // [0 <= alpha <= 0.5]

  // Optimization: No alpha blending when alpha is almost zero
  if (alpha < 0.01f) {
    return b;
  }

  auto normDisp = alphaBlend(alpha, b.normDisp, a.normDisp);
  auto normWeight = a.normWeight * normDispWeight(a.normDisp - normDisp) +
                    b.normWeight * normDispWeight(b.normDisp - normDisp);

  if (m_mode == Mode::depth) {
    return {normWeight, {}, normDisp, {}, {}};
  }
  return {normWeight, alphaBlend(alpha, b.color, a.color), normDisp,
          alphaBlend(alpha, b.rayAngle, a.rayAngle),
          alphaBlend(alpha, b.stretching, a.stretching)};
}

AccumulatingPixel::PixelValue
AccumulatingPixel::average(AccumulatingPixel::PixelAccumulator const &x) const {
  if (x.normWeight > 0) {
    return {x.color, 1.f / x.normDisp, x.normWeight,
            stretchingWeight(x.stretching)};
  }

  auto NaN = std::numeric_limits<float>::quiet_NaN();
  return {{0.f, 0.f, 0.f}, NaN, 0.f, 0.f};
}

void AccumulatingPixel::set(Vec2i point, PixelValue const &value, Mat3f &color,
                            Mat1f &depth, Mat1f &quality, Mat1f &validity) {
  depth(point.y(), point.x()) = value.depth;

  if (m_mode != Mode::depth) {
    color(point.y(), point.x()) = value.color;
    quality(point.y(), point.x()) = value.quality;
    validity(point.y(), point.x()) = value.validity;
  }
}

// Calculate the weight of a pixel based on cosine of the ray angle between
// input and virtual ray only
float AccumulatingPixel::rayAngleWeight(float rayAngle) const {
  return std::exp(-m_rayAngleParam * rayAngle);
}

// Calculate the weight of a pixel based on normalized disparity (diopters)
// only
float AccumulatingPixel::normDispWeight(float normDisp) const {
  return std::exp(m_depthParam * normDisp);
}

// Calculate the weight of a pixel based on stretching only
float AccumulatingPixel::stretchingWeight(float stretching) const {
  return std::exp(-m_stretchingParam * stretching);
}
} // namespace TMIV::Renderer
