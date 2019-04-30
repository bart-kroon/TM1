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

#ifndef _TMIV_RENDERER_RASTERIZER_H_
#error "Include the .h, not the .hpp"
#endif

#include "blend.h"
#include <TMIV/Common/Common.h>
#include <cmath>
#include <future>
#include <thread>

namespace TMIV::Renderer {
namespace {
// Calculate a number of strips that should ensure there is enough work but
// avoids having too many triangles in multiple strips
//
// Example: 8 hyper cores, 2048 rows ==> 128 strips of 16 rows each
int numStrips(int rows) {
  const double hw = std::thread::hardware_concurrency();
  const int maximum = (rows + 3) / 4;
  if (maximum <= hw) {
    return maximum;
  }
  return int(0.5 + sqrt(hw * maximum));
}

template <typename M0, typename... M>
auto fetchAttributes(int index, const std::tuple<M0, M...> &attributes) {
  std::tuple<typename M0::value_type, typename M::value_type...> result;
  std::get<0>(result) = std::get<0>(attributes)[index];
  if constexpr (sizeof...(M) >= 1) {
    std::get<1>(result) = std::get<1>(attributes)[index];
  }
  if constexpr (sizeof...(M) >= 2) {
    std::get<2>(result) = std::get<2>(attributes)[index];
  }
  if constexpr (sizeof...(M) >= 3) {
    std::get<3>(result) = std::get<3>(attributes)[index];
  }
  static_assert(sizeof...(M) <= 3);
  return result;
}

inline auto fetchAttributes(int /* index */, const std::tuple<> &
                            /* attributes */) -> std::tuple<> {
  return {};
}
} // namespace

template <typename... T>
Rasterizer<T...>::Rasterizer(Pixel pixel, Common::Vec2i size)
    : Rasterizer{pixel, size, numStrips(size.y())} {}

template <typename... T>
Rasterizer<T...>::Rasterizer(Pixel pixel, Common::Vec2i size, int numStrips)
    : m_pixel{pixel}, m_size{unsigned(size.y()), unsigned(size.x())} {
  assert(size.x() >= 0 && size.y() >= 0);
  assert(numStrips > 0);
  m_strips.reserve(numStrips);
  for (int n = 0; n < numStrips; ++n) {
    const int i1 = size.y() * n / numStrips;
    const int i2 = size.y() * (n + 1) / numStrips;
    m_strips.push_back(
        {i1,
         i2,
         size.x(),
         {},
         std::vector<Accumulator>{unsigned(i2 - i1) * size.x()}});
  }
  m_dk_di = float(numStrips) / float(size.y());
}

template <typename... T>
void Rasterizer<T...>::submit(ImageVertexDescriptorList vertices,
                              AttributeMaps attributes,
                              const TriangleDescriptorList &triangles) {
  m_batches.push_back(Batch{move(vertices), move(attributes)});
  for (auto &strip : m_strips) {
    strip.batches.emplace_back();
  }
  for (auto triangle : triangles) {
    submitTriangle(triangle, m_batches.back());
  }
}

template <typename... T> void Rasterizer<T...>::run() {
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

template <typename... T>
auto Rasterizer<T...>::depth() const -> Common::Mat<float> {
  Common::Mat<float> matrix(m_size);
  auto i_matrix = std::begin(matrix);
  visit([&i_matrix](const Value &x) {
    *i_matrix++ = x.depth();
    return true;
  });
  assert(i_matrix == std::end(matrix));
  return matrix;
}

template <typename... T>
auto Rasterizer<T...>::normDisp() const -> Common::Mat<float> {
  Common::Mat<float> matrix(m_size);
  auto i_matrix = std::begin(matrix);
  visit([&i_matrix](const Value &x) {
    *i_matrix++ = x.normDisp;
    return true;
  });
  assert(i_matrix == std::end(matrix));
  return matrix;
}

template <typename... T>
auto Rasterizer<T...>::normWeight() const -> Common::Mat<float> {
  Common::Mat<float> matrix(m_size);
  auto i_matrix = std::begin(matrix);
  visit([&i_matrix](const Value &x) {
    *i_matrix++ = x.normWeight;
    return true;
  });
  assert(i_matrix == std::end(matrix));
  return matrix;
}

template <typename... T>
template <size_t I>
auto Rasterizer<T...>::attribute() const
    -> Common::Mat<std::tuple_element_t<I, Attributes>> {
  Common::Mat<std::tuple_element_t<I, Attributes>> matrix(m_size);
  auto i_matrix = std::begin(matrix);
  visit([&i_matrix](const Value &x) {
    *i_matrix++ = std::get<I>(x.attributes());
    return true;
  });
  assert(i_matrix == std::end(matrix));
  return matrix;
}

template <typename... T>
template <class Visitor>
void Rasterizer<T...>::visit(Visitor visitor) const {
  if (!m_batches.empty()) {
    throw Exception{"The Rasterizer does not allow frame buffer access when "
                    "work is queued."};
  }
  for (const auto &strip : m_strips) {
    for (const auto &accumulator : strip.matrix) {
      if (!visitor(m_pixel.average(accumulator))) {
        return;
      }
    }
  }
}

template <typename... T>
void Rasterizer<T...>::submitTriangle(TriangleDescriptor descriptor,
                                      const Batch &batch) {
  const auto K = int(m_strips.size());
  auto k1 = K;
  auto k2 = 0;

  for (auto n : descriptor.indices) {
    const auto y = batch.vertices[n].position.y();
    if (std::isnan(y)) {
      return;
    }
    const auto k = y * m_dk_di;
    k1 = std::min(k1, TMIV::Common::ifloor(k));
    k2 = std::max(k2, TMIV::Common::iceil(k) + 1);
  }

  // Cull
  k1 = std::max(0, k1);
  k2 = std::min(K, k2);

  for (int k = k1; k < k2; ++k) {
    m_strips[k].batches.back().push_back(descriptor);
  }
}

template <typename... T>
void Rasterizer<T...>::rasterTriangle(TriangleDescriptor descriptor,
                                      const Batch &batch, Strip &strip) {
  const auto n0 = descriptor.indices[0];
  const auto n1 = descriptor.indices[1];
  const auto n2 = descriptor.indices[2];

  // Image coordinate within strip
  const auto uv0 =
      batch.vertices[n0].position - Common::Vec2f{0.f, float(strip.i1)};
  const auto uv1 =
      batch.vertices[n1].position - Common::Vec2f{0.f, float(strip.i1)};
  const auto uv2 =
      batch.vertices[n2].position - Common::Vec2f{0.f, float(strip.i1)};

  // Determine triangle bounding box
  const auto u1 = std::max(0, TMIV::Common::ifloor(std::min({uv0.x(), uv1.x(), uv2.x()})));
  const auto u2 =
      std::min(strip.cols, 1 + TMIV::Common::iceil(std::max({uv0.x(), uv1.x(), uv2.x()})));
  if (u1 >= u2) {
    return; // Cull
  }
  const auto v1 = std::max(0, TMIV::Common::ifloor(std::min({uv0.y(), uv1.y(), uv2.y()})));
  const auto v2 =
      std::min(strip.rows(), 1 + TMIV::Common::iceil(std::max({uv0.y(), uv1.y(), uv2.y()})));
  if (v1 >= v2) {
    return; // Cull
  }

  // Determine (unclipped) parallelogram area
  const auto area = ((uv1.y() - uv2.y()) * (uv0.x() - uv2.x()) +
                     (uv2.x() - uv1.x()) * (uv0.y() - uv2.y()));
  if (area <= 0.f) {
    return; // Cull
  }
  const auto inv_area = 1.f / area;

  // Calculate feature values for determining blending weights
  const auto stretching = 0.5f * area / descriptor.area;
  const auto rayAngle =
      (1 / 3.f) * (batch.vertices[n0].rayAngle + batch.vertices[n1].rayAngle +
                   batch.vertices[n2].rayAngle);

  // Fetch normalized disparity values (diopters)
  const auto d0 = 1.f / batch.vertices[n0].depth;
  const auto d1 = 1.f / batch.vertices[n1].depth;
  const auto d2 = 1.f / batch.vertices[n2].depth;

  // Fetch multiple attributes (e.g. color)
  const auto a0 = fetchAttributes(n0, batch.attributes);
  const auto a1 = fetchAttributes(n1, batch.attributes);
  const auto a2 = fetchAttributes(n2, batch.attributes);

  // For each pixel in the bounding box
  for (int v = v1; v < v2; ++v) {
    for (int u = u1; u < u2; ++u) {
      // Small epsilon value to avoid skipping pixels when the grids aligns.
      // This happens when synthesizing from and to the same camera. This will
      // not happen for arbitrary viewports but may happens on the Encoder side.
      // TODO: Use fixed point image positions to avoid this problem
      const float eps = 1e-6f;

      // Calculate the Barycentric coordinate of the pixel center (x +
      // 1/2, y + 1/2)
      const float w0 =
          inv_area * ((uv1.y() - uv2.y()) * (float(u) - uv2.x() + 0.5f) +
                      (uv2.x() - uv1.x()) * (float(v) - uv2.y() + 0.5f));
      if (!(w0 >= -eps)) {
        continue;
      }
      const float w1 =
          inv_area * ((uv2.y() - uv0.y()) * (float(u) - uv2.x() + 0.5f) +
                      (uv0.x() - uv2.x()) * (float(v) - uv2.y() + 0.5f));
      if (!(w1 >= -eps)) {
        continue;
      }
      const float w2 = 1.f - w0 - w1;
      if (!(w2 >= -eps)) {
        continue;
      }

      // Barycentric interpolation of normalized disparity and attributes
	  // (e.g. color)
      const auto d = w0 * d0 + w1 * d1 + w2 * d2;
      const auto a = blendAttributes(w0, a0, w1, a1, w2, a2);

      // Blend pixel
      assert(v * strip.cols + u < int(strip.matrix.size()));
      auto &P = strip.matrix[v * strip.cols + u];
      P = m_pixel.blend(P, m_pixel.construct(a, d, rayAngle, stretching));
    }
  }
}

template <typename... T> void Rasterizer<T...>::clearBatches() {
  for (auto &strip : m_strips) {
    strip.batches.clear();
  }
  m_batches.clear();
}
} // namespace TMIV::Renderer
