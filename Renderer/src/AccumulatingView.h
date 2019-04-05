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

#ifndef _TMIV_RENDERING_ACCUMULATINGVIEW_H_
#define _TMIV_RENDERING_ACCUMULATINGVIEW_H_

#include "AccumulatingPixel.h"

namespace TMIV::Renderer {
enum class WrappingMethod { none = 0, horizontal = 1 };

// View synthesis and blending based on AccumulatingPixel
class AccumulatingView {
private:
  using Vec2i = Common::Vec2i;
  using Vec2f = Common::Vec2f;
  using Vec3f = Common::Vec3f;
  using Mat3f = Common::Mat<Vec3f>;
  using Mat2f = Common::Mat<Vec2f>;
  using Mat1f = Common::Mat<float>;
  using PixelAccumulator = AccumulatingPixel::PixelAccumulator;

public:
  using Mode = AccumulatingPixel::Mode;

  /**
   * RayAngle: The angle [rad] between the ray from the input camera and the ray
   * from the virtual camera: exp(-RayAngleParameter × RayAngle). Prefer nearby
   * views over views further away (soft view selection).
   *
   * Depth: The depth value [meter] but the reciprocal [diopter] is used in
   * the equation: exp(DepthParameter / Depth) Prefer foreground over background
   * (depth ordering).
   *
   * Stretching: The maximal side [pixel] of the triangle after reprojection,
   * with a compensation for ERP: exp(-StretchingParameter × Stretching)
   * Downweight triangles that stretch between foreground and background
   * objects.
   */
  AccumulatingView(double rayAngleParam, double depthParam,
                   double stretchingParam, Mode mode);

  // Synthesized and blended texture
  Mat3f texture() const;

  // Syntesized and blended depth map (depth values are meters)
  Mat1f depth() const;

  // Syntesized and blended depth map (depth values are diopters)
  Mat1f normDisp() const;

  // Quality of blended result (based on ray angle, depth and stretching of
  // contributions)
  Mat1f quality() const;

  // Validity of blended result (based on stretching of contributions)
  Mat1f validity() const;

  // Blend in another view
  AccumulatingView &blendWith(AccumulatingView const &other);

  /**
   * Rasterize the warped image, resulting in updates of color, depth and
   * quality maps.
   *
   * Accumulates the data using exponential weighting of depth and stretching
   * Same blending algorithm and parameters within and accross views
   * Accumulates the data of this single view
   * Calls average()
   */
  void transform(const Mat3f &texture, const Mat2f &positions,
                 const Mat1f &depth, const Mat1f &rayAngles, Vec2i outputSize,
                 WrappingMethod wrappingMethod);

private:
  template <Mode mode>
  void colorizeTriangle(const Mat3f &color, const Mat1f &depth,
                        const Mat2f &positions, const Mat1f &rayAngles, Vec2i a,
                        Vec2i b, Vec2i c);

  template <Mode mode>
  void colorizeSquare(const Mat3f &color, const Mat1f &depth,
                      const Mat2f &positions, const Mat1f &rayAngles, Vec2i TL,
                      Vec2i TR, Vec2i BR, Vec2i BL);

  AccumulatingPixel m_pixel;
  Common::Mat<PixelAccumulator> m_sums;
};
} // namespace TMIV::Renderer

#endif