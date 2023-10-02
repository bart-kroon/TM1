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

#include <TMIV/Renderer/Rasterizer.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/Thread.h>

#include <cmath>
#include <future>

namespace TMIV::Renderer {
namespace {
// Calculate a number of strips that should ensure there is enough work but
// avoids having too many triangles in multiple strips
//
// Example: 8 hyper cores, 2048 rows ==> 128 strips of 16 rows each
inline auto numStrips(int32_t rows) -> int32_t {
  const double hw = Common::threadCount();
  const int32_t maximum = (rows + 3) / 4;
  if (maximum <= hw) {
    return maximum;
  }
  using std::sqrt;
  return static_cast<int32_t>(std::lround(sqrt(hw * maximum)));
}
} // namespace

Rasterizer::Rasterizer(AccumulatingPixel pixel, Common::Vec2i size)
    : Rasterizer{pixel, size, numStrips(size.y())} {}

Rasterizer::Rasterizer(AccumulatingPixel pixel, Common::Vec2i size, int32_t numStrips)
    : m_pixel{pixel}, m_size{static_cast<size_t>(size.y()), static_cast<size_t>(size.x())} {
  PRECONDITION(size.x() >= 0 && size.y() >= 0);
  PRECONDITION(numStrips > 0);
  m_strips.reserve(numStrips);
  for (int32_t n = 0; n < numStrips; ++n) {
    const auto i1 = size.y() * n / numStrips;
    const auto i2 = size.y() * (n + 1) / numStrips;
    auto accumulator =
        std::vector<PixelAccumulator>{static_cast<size_t>(i2 - i1) * static_cast<size_t>(size.x())};
    m_strips.push_back({i1, i2, size.x(), {}, std::move(accumulator)});
  }
  m_dk_di = static_cast<float>(numStrips) / static_cast<float>(size.y());
}

void Rasterizer::submit(const ImageVertexDescriptorList &vertices, std::vector<Common::Vec3f> color,
                        const TriangleDescriptorList &triangles) {
  m_batches.push_back(Batch{vertices, std::move(color)});
  for (auto &strip : m_strips) {
    strip.batches.emplace_back();
  }
  for (auto triangle : triangles) {
    submitTriangle(triangle, m_batches.back());
  }
}

void Rasterizer::run() {
  std::vector<std::future<void>> work;
  work.reserve(m_strips.size());

  // Launch all work
  for (auto &strip : m_strips) {
    work.push_back(std::async( // Strips in parallel
        [this, &strip]() {
          for (size_t i = 0; i < m_batches.size(); ++i) { // Batches in sequence
            for (auto triangle : strip.batches[i]) {
              rasterTriangle(triangle, m_batches[i], strip);
            }
          }
        }));
  }

  // Synchronize on completion
  for (auto &item : work) {
    item.get();
  }

  // Deallocate
  clearBatches();
}

auto Rasterizer::depth() const -> Common::Mat<float> {
  Common::Mat<float> matrix(m_size);
  auto i_matrix = std::begin(matrix);
  visit([&i_matrix](const PixelValue &x) {
    *i_matrix++ = x.depth();
    return true;
  });
  POSTCONDITION(i_matrix == std::end(matrix));
  return matrix;
}

auto Rasterizer::normDisp() const -> Common::Mat<float> {
  Common::Mat<float> matrix(m_size);
  auto i_matrix = std::begin(matrix);
  visit([&i_matrix](const PixelValue &x) {
    *i_matrix++ = x.normDisp;
    return true;
  });
  POSTCONDITION(i_matrix == std::end(matrix));
  return matrix;
}

auto Rasterizer::normWeight() const -> Common::Mat<float> {
  Common::Mat<float> matrix(m_size);
  auto i_matrix = std::begin(matrix);
  visit([&i_matrix](const PixelValue &x) {
    *i_matrix++ = x.normWeight;
    return true;
  });
  POSTCONDITION(i_matrix == std::end(matrix));
  return matrix;
}

void Rasterizer::submitTriangle(TriangleDescriptor descriptor, const Batch &batch) {
  const auto K = static_cast<int32_t>(m_strips.size());
  auto k1 = K;
  auto k2 = 0;

  for (auto n : descriptor.indices) {
    const auto y = batch.vertices[n].position.y();
    if (std::isnan(y)) {
      return;
    }
    const auto k = y * m_dk_di;
    k1 = std::min(k1, static_cast<int32_t>(std::floor(k)));
    k2 = std::max(k2, static_cast<int32_t>(std::ceil(k)) + 1);
  }

  // Cull
  k1 = std::max(0, k1);
  k2 = std::min(K, k2);

  for (int32_t k = k1; k < k2; ++k) {
    m_strips[k].batches.back().push_back(descriptor);
  }
}

// Switch to fixed-point vertices to correctly handle edge points
namespace {
using intfp = int_fast32_t;
using Vec2fp = Common::stack::Vec2<intfp>;

constexpr const auto bits = intfp{4};
constexpr const auto eps = intfp{1};
constexpr const auto one = eps << bits;
constexpr const auto half = one / intfp{2};

inline auto fixed(float x) noexcept -> intfp {
  using std::ldexp;
  return static_cast<intfp>(std::floor(0.5F + ldexp(x, bits)));
}

inline auto fixed(Common::Vec2f v) noexcept { return Vec2fp{fixed(v.x()), fixed(v.y())}; }
constexpr auto fixed(int32_t x) noexcept { return static_cast<intfp>(x) << bits; }
constexpr auto fpfloor(intfp x) noexcept { return static_cast<int32_t>(x >> bits); }
constexpr auto fpceil(intfp x) noexcept { return fpfloor(x + one - eps); }

struct TriangleInfo {
  int32_t u1;
  int32_t u2;
  int32_t v1;
  int32_t v2;
  intfp area;
  float invArea;
};

auto determineTriangleBoundingBoxAndArea(int32_t rows, int32_t cols,
                                         const std::array<Vec2fp, 3> &uv) noexcept
    -> std::optional<TriangleInfo> {
  auto info = TriangleInfo{};

  // Determine triangle bounding box
  info.u1 = std::max(0, fpfloor(std::min({uv[0].x(), uv[1].x(), uv[2].x()})));
  info.u2 = std::min(cols, 1 + fpceil(std::max({uv[0].x(), uv[1].x(), uv[2].x()})));
  if (info.u1 >= info.u2) {
    return std::nullopt; // Cull
  }
  info.v1 = std::max(0, fpfloor(std::min({uv[0].y(), uv[1].y(), uv[2].y()})));
  info.v2 = std::min(rows, 1 + fpceil(std::max({uv[0].y(), uv[1].y(), uv[2].y()})));
  if (info.v1 >= info.v2) {
    return std::nullopt; // Cull
  }

  // Determine (unclipped) parallelogram area
  info.area = (uv[1].y() - uv[2].y()) * (uv[0].x() - uv[2].x()) +
              (uv[2].x() - uv[1].x()) * (uv[0].y() - uv[2].y());
  if (info.area <= 0) {
    return std::nullopt; // Cull
  }
  info.invArea = 1.F / static_cast<float>(info.area);
  return info;
}

// Calculate the Barycentric coordinate of the pixel center
// (u + 1/2, v + 1/2)
auto calculateBarycentricCoordinate(int32_t u, int32_t v, const TriangleInfo &info,
                                    const std::array<Vec2fp, 3> &uv)
    -> std::optional<std::array<float, 3>> {
  const auto X0 = (uv[1].y() - uv[2].y()) * (fixed(u) - uv[2].x() + half) +
                  (uv[2].x() - uv[1].x()) * (fixed(v) - uv[2].y() + half);
  if (X0 < 0) {
    return std::nullopt;
  }
  const auto X1 = (uv[2].y() - uv[0].y()) * (fixed(u) - uv[2].x() + half) +
                  (uv[0].x() - uv[2].x()) * (fixed(v) - uv[2].y() + half);
  if (X1 < 0) {
    return std::nullopt;
  }
  const auto X2 = info.area - X0 - X1;
  if (X2 < 0) {
    return std::nullopt;
  }

  return std::array{info.invArea * static_cast<float>(X0), info.invArea * static_cast<float>(X1),
                    info.invArea * static_cast<float>(X2)};
}

// Blend three arithmetic tensors of fixed size
template <typename T> auto blendValues(float w_a, T a, float w_b, T b, float w_c, T c) -> T {
  if constexpr (std::is_floating_point_v<T>) {
    return w_a * a + w_b * b + w_c * c;
  } else if constexpr (std::is_integral_v<T>) {
    return static_cast<T>(std::lround(w_a * static_cast<float>(a) + w_b * static_cast<float>(b) +
                                      w_c * static_cast<float>(c)));
  } else {
    T result;
    static_assert(result.size() == a.size()); // req. constexpr size()
    for (uint32_t i = 0; i < result.size(); ++i) {
      result[i] = blendValues(w_a, a[i], w_b, b[i], w_c, c[i]);
    }
    return result;
  }
}
} // namespace

void Rasterizer::rasterTriangle(TriangleDescriptor descriptor, const Batch &batch, Strip &strip) {
  using std::ldexp;
  using std::max;
  using std::min;

  const auto n0 = descriptor.indices[0];
  const auto n1 = descriptor.indices[1];
  const auto n2 = descriptor.indices[2];

  // Image coordinate within strip
  const auto stripOffset = Vec2fp{0, fixed(strip.i1)};
  const auto uv = std::array{fixed(batch.vertices[n0].position) - stripOffset,
                             fixed(batch.vertices[n1].position) - stripOffset,
                             fixed(batch.vertices[n2].position) - stripOffset};

  if (const auto triangleInfo = determineTriangleBoundingBoxAndArea(strip.rows(), strip.cols, uv)) {
    const auto area_f = ldexp(static_cast<float>(triangleInfo->area), -2 * bits);

    // Calculate feature values for determining blending weights
    const auto stretching = 0.5F * area_f / descriptor.area;
    const auto rayAngle = (1 / 3.F) * (batch.vertices[n0].rayAngle + batch.vertices[n1].rayAngle +
                                       batch.vertices[n2].rayAngle);

    // Fetch normalized disparity values (diopters)
    const auto d0 = 1.F / batch.vertices[n0].depth;
    const auto d1 = 1.F / batch.vertices[n1].depth;
    const auto d2 = 1.F / batch.vertices[n2].depth;

    // Fetch multiple attributes (e.g. color)
    const auto a0 = batch.color[n0];
    const auto a1 = batch.color[n1];
    const auto a2 = batch.color[n2];

    // For each pixel in the bounding box
    for (int32_t v = triangleInfo->v1; v < triangleInfo->v2; ++v) {
      for (int32_t u = triangleInfo->u1; u < triangleInfo->u2; ++u) {
        if (const auto coord = calculateBarycentricCoordinate(u, v, *triangleInfo, uv)) {
          const auto [w0, w1, w2] = *coord;

          // Barycentric interpolation of normalized disparity and attributes
          // (e.g. color)
          const auto d = w0 * d0 + w1 * d1 + w2 * d2;
          const auto a = blendValues(w0, a0, w1, a1, w2, a2);

          // Blend pixel
          ASSERT(v * strip.cols + u < static_cast<int32_t>(strip.matrix.size()));
          auto &P = strip.matrix[v * strip.cols + u];

          auto p = m_pixel.construct(a, d, rayAngle, stretching);
          if (w0 == 0.F || w1 == 0.F || w2 == 0.F) {
            // Count edge points half assuming there is an adjacent triangle
            p.normWeight *= 0.5F;
          }
          P = m_pixel.blend(P, p);
        }
      }
    }
  }
}

void Rasterizer::clearBatches() {
  for (auto &strip : m_strips) {
    strip.batches.clear();
  }
  m_batches.clear();
}
} // namespace TMIV::Renderer
