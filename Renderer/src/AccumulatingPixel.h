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

#ifndef _TMIV_RENDERER_ACCUMULATINGPIXEL_H_
#define _TMIV_RENDERER_ACCUMULATINGPIXEL_H_

#include <TMIV/Common/Matrix.h>
#include <TMIV/Common/Vector.h>

namespace TMIV::Renderer {
// Per-pixel operations for AccumulatingView
class AccumulatingPixel {
public:
  using Vec2i = Common::Vec2i;
  using Vec3f = Common::Vec3f;
  using Mat3f = Common::Mat<Vec3f>;
  using Mat1f = Common::Mat<float>;

  AccumulatingPixel(double rayAngleParam, double depthParam,
                    double stretchingParam);

  struct PixelAccumulator {
    // weight is implicit as norm_weight * normDispWeight(1 / depth)
    // but never directly calculated to avoid numerical instability.
    float normWeight{0.f};

    // TODO: The following fields could be made 16-bit float to save on memory
    Vec3f color;
    float normDisp{0.f};
    float rayAngle{0.f}; // for sampling/tuning
    float stretching{0.f};
  };

  // Construct a pixel accumulator from the synthesis of a single input pixel
  PixelAccumulator construct(float pixelDepth, Vec3f pixelColor, float rayAngle,
                             float stretching) const;

  // Blend two pixels
  PixelAccumulator blend(const PixelAccumulator &a,
                         const PixelAccumulator &b) const;

  struct PixelValue {
    Vec3f color;
    float depth;
    float quality;
    float validity;
  };

  // Average a pixel
  PixelValue average(PixelAccumulator const &x) const;

  // Set a pixel in a set of value maps
  void set(Vec2i x, PixelValue const &value, Mat3f &color, Mat1f &depth,
           Mat1f &quality, Mat1f &validity);

  // Calculate the weight of a pixel based on cosine of the ray angle between
  // input and virtual ray only
  float rayAngleWeight(float rayAngle) const;

  // Calculate the weight of a pixel based on normalized disparity (diopters)
  // only
  float normDispWeight(float normDisp) const;

  // Calculate the weight of a pixel based on stretching only
  float stretchingWeight(float stretching) const;

private:
  float m_rayAngleParam;
  float m_depthParam;
  float m_stretchingParam;
};
} // namespace TMIV::Renderer

#endif