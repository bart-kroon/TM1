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

#ifndef TMIV_RENDERER_PUSHPULL_H
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/Frame.h>

namespace TMIV::Renderer {
template <typename PushFilter, typename PullFilter>
auto PushPull::filter(const Common::RendererFrame &frame, PushFilter &&pushFilter,
                      PullFilter &&pullFilter) -> const Common::RendererFrame & {
  m_pyramid.push_back(frame);
  while (1 < m_pyramid.back().texture.getWidth() || 1 < m_pyramid.back().texture.getHeight()) {
    const auto w = (m_pyramid.back().texture.getWidth() + 1) / 2;
    const auto h = (m_pyramid.back().texture.getHeight() + 1) / 2;
    m_pyramid.emplace_back().texture = Common::Frame<>::yuv444({w, h}, frame.texture.getBitDepth());
    m_pyramid.back().geometry = Common::Frame<>::lumaOnly({w, h}, frame.geometry.getBitDepth());
    auto i = m_pyramid.rbegin();
    inplacePushFrame(*(i + 1), *i, pushFilter);
  }

  for (auto i = m_pyramid.rbegin(); i + 1 != m_pyramid.rend(); ++i) {
    inplacePullFrame(*i, *(i + 1), pullFilter);
  }

  return m_pyramid.front();
}

inline auto PushPull::numLayers() const noexcept { return m_pyramid.size(); }

inline auto PushPull::layer(size_t i) const noexcept -> const Common::RendererFrame & {
  return m_pyramid[i];
}

inline auto PushPull::yuvd(const Common::RendererFrame &frame) {
  return [plane = std::array<std::reference_wrapper<const Common::Mat<uint16_t>>, 4>{
              frame.texture.getPlane(0), frame.texture.getPlane(1), frame.texture.getPlane(2),
              frame.geometry.getPlane(0)}](int32_t i, int32_t j) {
    return std::tuple{plane[0](i, j), plane[1](i, j), plane[2](i, j), plane[3](i, j)};
  };
}

inline auto PushPull::yuvd(Common::RendererFrame &frame) {
  return [plane = std::array<std::reference_wrapper<Common::Mat<uint16_t>>, 4>{
              frame.texture.getPlane(0), frame.texture.getPlane(1), frame.texture.getPlane(2),
              frame.geometry.getPlane(0)}](int32_t i, int32_t j) {
    return std::tie(plane[0](i, j), plane[1](i, j), plane[2](i, j), plane[3](i, j));
  };
}

template <typename PushFilter>
void PushPull::inplacePushFrame(const Common::RendererFrame &in, Common::RendererFrame &out,
                                PushFilter &&filter) {
  const auto wi = in.texture.getWidth();
  const auto hi = in.texture.getHeight();
  const auto wo = (wi + 1) / 2;
  const auto ho = (hi + 1) / 2;
  out.texture.createYuv444({wo, ho}, in.texture.getBitDepth());
  out.geometry.createY({wo, ho}, in.geometry.getBitDepth());
  return inplacePush(MatrixProxy{yuvd(in), wi, hi}, MatrixProxy{yuvd(out), wo, ho},
                     std::forward<PushFilter>(filter));
}

template <typename PullFilter>
void PushPull::inplacePullFrame(const Common::RendererFrame &in, Common::RendererFrame &out,
                                PullFilter &&filter) noexcept {
  const auto wi = in.texture.getWidth();
  const auto hi = in.texture.getHeight();
  const auto wo = out.texture.getWidth();
  const auto ho = out.texture.getHeight();
  return inplacePull(MatrixProxy{yuvd(in), wi, hi}, MatrixProxy{yuvd(out), wo, ho},
                     std::forward<PullFilter>(filter));
}

template <typename InMatrixProxy, typename OutMatrixProxy, typename PushFilter>
void PushPull::inplacePush(const InMatrixProxy &&in, OutMatrixProxy &&out,
                           PushFilter &&filter) noexcept {
  PRECONDITION(out.width == (in.width + 1) / 2);
  PRECONDITION(out.height == (in.height + 1) / 2);

  for (int32_t y = 0; y < out.height; ++y) {
    const auto y_1 = 2 * y;
    const auto y_2 = std::min(2 * y + 1, in.height - 1); // repeat bottom border

    for (int32_t x = 0; x < out.width; ++x) {
      const auto x_1 = 2 * x;
      const auto x_2 = std::min(2 * x + 1, in.width - 1); // repeat right border

      out.matrix(y, x) = filter(std::array{in.matrix(y_1, x_1), in.matrix(y_1, x_2),
                                           in.matrix(y_2, x_1), in.matrix(y_2, x_2)});
    }
  }
}

template <typename InMatrixProxy, typename OutMatrixProxy, typename PullFilter>
void PushPull::inplacePull(const InMatrixProxy &&in, OutMatrixProxy &&out,
                           PullFilter &&filter) noexcept {
  PRECONDITION(in.width == (out.width + 1) / 2);
  PRECONDITION(in.height == (out.height + 1) / 2);

  for (int32_t y = 0; y < out.height; ++y) {
    auto y_1 = std::max(0, (y - 1) / 2);
    auto y_2 = std::min(in.height - 1, (y + 1) / 2);
    if (y % 2 == 0) {
      std::swap(y_1, y_2);
    }

    for (int32_t x = 0; x < out.width; ++x) {
      auto x_1 = std::max(0, (x - 1) / 2);
      auto x_2 = std::min(in.width - 1, (x + 1) / 2);
      if (x % 2 == 0) {
        std::swap(x_1, x_2);
      }
      const auto value = out.matrix(y, x);

      out.matrix(y, x) = filter(std::array{in.matrix(y_1, x_1), in.matrix(y_1, x_2),
                                           in.matrix(y_2, x_1), in.matrix(y_2, x_2)},
                                value);
    }
  }
}
} // namespace TMIV::Renderer
