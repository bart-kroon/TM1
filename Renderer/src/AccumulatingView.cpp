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

#include "AccumulatingView.h"

#include <algorithm>
#include <cassert>
#include <iterator>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::Renderer {
AccumulatingView::AccumulatingView(double rayAngleParam, double depthParam,
                                   double stretchingParam)
    : m_pixel{rayAngleParam, depthParam, stretchingParam} {}

AccumulatingView::Mat1f AccumulatingView::depth() const {
  assert(!m_sums.empty());
  Mat1f result(m_sums.sizes());
  std::transform(
      begin(m_sums), end(m_sums), begin(result),
      [this](const PixelAccumulator &acc) { return 1.f / acc.normDisp; });
  return result;
}

AccumulatingView::Mat1f AccumulatingView::normDisp() const {
  assert(!m_sums.empty());
  Mat1f result(m_sums.sizes());
  std::transform(begin(m_sums), end(m_sums), begin(result),
                 [this](const PixelAccumulator &acc) { return acc.normDisp; });
  return result;
}

AccumulatingView &AccumulatingView::blendWith(AccumulatingView const &other) {
  if (m_sums.empty()) {
    m_sums = other.m_sums;
  } else {
    assert(m_sums.size() == other.m_sums.size());
    std::transform(
        cbegin(m_sums), cend(m_sums), cbegin(other.m_sums), begin(m_sums),
        [this](const PixelAccumulator &a, const PixelAccumulator &b) {
          return m_pixel.blend(a, b);
        });
  }
  return *this;
}

void AccumulatingView::transform(const Mat3f &texture, const Mat2f &positions,
                                 const Mat1f &depth, const Mat1f &rayAngles,
                                 Vec2i outputSize,
                                 WrappingMethod wrappingMethod) {
  if (m_sums.empty()) {
    m_sums.resize(outputSize.y(), outputSize.x());
    m_sums = PixelAccumulator{};
  }
  assert(static_cast<int>(m_sums.height()) == outputSize.y() &&
         static_cast<int>(m_sums.width()) == outputSize.x());

  const int height = static_cast<int>(texture.height());
  const int width = static_cast<int>(texture.width());

  // Raster two triangles per pixel
  for (int i = 0; i < height - 1; ++i) {
    for (int j = 0; j < width - 1; ++j) {
      auto TL = Vec2i{j, i};
      auto TR = Vec2i{j + 1, i};
      auto BL = Vec2i{j, i + 1};
      auto BR = Vec2i{j + 1, i + 1};
      colorizeSquare(texture, depth, positions, rayAngles, TL, TR, BR, BL);
    }

    // Stitch left and right borders with triangles (e.g. for
    // equirectangular projection)
    // TODO: Solve this is a better way
    if (wrappingMethod == WrappingMethod::horizontal) {
      auto TL = Vec2i{width - 1, i};
      auto TR = Vec2i{0, i};
      auto BL = Vec2i{width - 1, i + 1};
      auto BR = Vec2i{0, i + 1};
      colorizeSquare(texture, depth, positions, rayAngles, TL, TR, BR, BL);
    }
  }
}

void AccumulatingView::colorizeSquare(const Mat3f &color, const Mat1f &depth,
                                      const Mat2f &input_positions,
                                      const Mat1f &rayAngles, Vec2i TL,
                                      Vec2i TR, Vec2i BR, Vec2i BL) {
  if (depth(TR.y(), TR.x()) > 0.f && depth(BL.y(), BL.x()) > 0.f) {
    if (depth(TL.y(), TL.x()) > 0.f) {
      colorizeTriangle(color, depth, input_positions, rayAngles, TL, TR, BL);
    }
    if (depth(BR.y(), BR.x()) > 0.f) {
      colorizeTriangle(color, depth, input_positions, rayAngles, BR, BL, TR);
    }
  }
}

void AccumulatingView::colorizeTriangle(const Mat3f &color, const Mat1f &depth,
                                        const Mat2f &positions,
                                        const Mat1f &rayAngles, Vec2i a,
                                        Vec2i b, Vec2i c) {
  Vec2f A = positions(a.y(), a.x());
  Vec2f B = positions(b.y(), b.x());
  Vec2f C = positions(c.y(), c.x());

  float den = (B[1] - C[1]) * (A[0] - C[0]) + (C[0] - B[0]) * (A[1] - C[1]);
  if (den <= 0.f) {
    return; // Do not draw backside or edge of triangle
  }

  // Determine triangle bounding box
  auto Xmin = max(0, static_cast<int>(floor(min(min(A[0], B[0]), C[0]))));
  auto Ymin = max(0, static_cast<int>(floor(min(min(A[1], B[1]), C[1]))));
  auto Xmax = min(static_cast<int>(m_sums.width()) - 1,
                  static_cast<int>(ceil(max(max(A[0], B[0]), C[0]))));
  auto Ymax = min(static_cast<int>(m_sums.height()) - 1,
                  static_cast<int>(ceil(max(max(A[1], B[1]), C[1]))));
  if (Ymin > Ymax || Xmin > Xmax) {
    return; // Cull triangle (optimization)
  }

  // Determine amount of stretching of the triangle,
  auto stretching = max({norm(A - B), norm(A - C), norm(B - C)});

  // Cosine of ray angles between input and virtual camera
  auto ray_angle = min({rayAngles(a.y(), a.x()), rayAngles(b.y(), b.x()),
                        rayAngles(c.y(), c.x())});

  // Fetch color and depth values
  auto colorA = color(a.y(), a.x());
  auto colorB = color(b.y(), b.x());
  auto colorC = color(c.y(), c.x());
  auto depthA = depth(a.y(), a.x());
  auto depthB = depth(b.y(), b.x());
  auto depthC = depth(c.y(), c.x());

  // For each pixel in the bounding box
  for (int y = Ymin; y <= Ymax; ++y) {
    for (int x = Xmin; x <= Xmax; ++x) {
      // Calculate the Barycentric coordinate of the pixel center (x +
      // 1/2, y + 1/2)
      auto x_center = static_cast<float>(x) + 0.5f;
      auto y_center = static_cast<float>(y) + 0.5f;
      float lambdaA = ((B[1] - C[1]) * (x_center - C[0]) +
                       (C[0] - B[0]) * (y_center - C[1])) /
                      den;
      float lambdaB = ((C[1] - A[1]) * (x_center - C[0]) +
                       (A[0] - C[0]) * (y_center - C[1])) /
                      den;
      float lambdaC = 1.f - lambdaA - lambdaB;

      // If on the edge or inside the triangle...
      float const eps = 1e-6f;
      if (lambdaA >= -eps && lambdaB >= -eps && lambdaC >= -eps) {
        // Barycentric interpolation of depth and color
        auto pixelDepth =
            depthA * lambdaA + depthB * lambdaB + depthC * lambdaC;
        auto pixelColor =
            colorA * lambdaA + colorB * lambdaB + colorC * lambdaC;

        // Construct pixel accumulator
        auto accum =
            m_pixel.construct(pixelDepth, pixelColor, ray_angle, stretching);

        // Blend
        m_sums(y, x) = m_pixel.blend(m_sums(y, x), accum);
      }
    }
  }
}
} // namespace TMIV::Renderer
