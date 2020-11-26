/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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

#ifndef _TMIV_RENDERER_PUSHPULL_H_
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/Frame.h>

namespace TMIV::Renderer {
template <typename PushFilter, typename PullFilter>
auto PushPull::filter(const Common::Texture444Depth16Frame &frame, PushFilter &&pushFilter,
                      PullFilter &&pullFilter) -> const Common::Texture444Depth16Frame & {
  m_pyramid.push_back(frame);
  while (1 < m_pyramid.back().first.getWidth() || 1 < m_pyramid.back().first.getHeight()) {
    const auto w = (m_pyramid.back().first.getWidth() + 1) / 2;
    const auto h = (m_pyramid.back().first.getHeight() + 1) / 2;
    m_pyramid.emplace_back(Common::Texture444Frame{w, h}, Common::Depth16Frame{w, h});
    auto i = m_pyramid.rbegin();
    inplacePush(*(i + 1), *i, pushFilter);
  }

  for (auto i = m_pyramid.rbegin(); i + 1 != m_pyramid.rend(); ++i) {
    inplacePull(*i, *(i + 1), pullFilter);
  }

  return m_pyramid.front();
}

inline auto PushPull::numLayers() const noexcept -> int {
  return static_cast<int>(m_pyramid.size());
}

inline auto PushPull::layer(int i) const noexcept -> const Common::Texture444Depth16Frame & {
  return m_pyramid[i];
}

inline auto PushPull::yuvd(const Common::Texture444Depth16Frame &frame) {
  return [plane = std::array<std::reference_wrapper<const Common::Mat<uint16_t>>, 4>{
              frame.first.getPlane(0), frame.first.getPlane(1), frame.first.getPlane(2),
              frame.second.getPlane(0)}](int i, int j) {
    return std::tuple{plane[0](i, j), plane[1](i, j), plane[2](i, j), plane[3](i, j)};
  };
}

inline auto PushPull::yuvd(Common::Texture444Depth16Frame &frame) {
  return [plane = std::array<std::reference_wrapper<Common::Mat<uint16_t>>, 4>{
              frame.first.getPlane(0), frame.first.getPlane(1), frame.first.getPlane(2),
              frame.second.getPlane(0)}](int i, int j) {
    return std::tie(plane[0](i, j), plane[1](i, j), plane[2](i, j), plane[3](i, j));
  };
}

template <typename PushFilter>
void PushPull::inplacePush(const Common::Texture444Depth16Frame &in,
                           Common::Texture444Depth16Frame &out, PushFilter &&filter) {
  const auto wi = in.first.getWidth();
  const auto hi = in.first.getHeight();
  const auto wo = (wi + 1) / 2;
  const auto ho = (hi + 1) / 2;
  out.first.resize(wo, ho);
  out.second.resize(wo, ho);
  return inplacePush(yuvd(in), wi, hi, yuvd(out), wo, ho, std::forward<PushFilter>(filter));
}

template <typename PullFilter>
void PushPull::inplacePull(const Common::Texture444Depth16Frame &in,
                           Common::Texture444Depth16Frame &out, PullFilter &&filter) noexcept {
  const auto wi = in.first.getWidth();
  const auto hi = in.first.getHeight();
  const auto wo = out.first.getWidth();
  const auto ho = out.first.getHeight();
  return inplacePull(yuvd(in), wi, hi, yuvd(out), wo, ho, std::forward<PullFilter>(filter));
}

template <typename InMatrix, typename OutMatrix, typename PushFilter>
void PushPull::inplacePush(const InMatrix &&in, int wi, int hi, OutMatrix &&out, int wo, int ho,
                           PushFilter &&filter) noexcept {
  assert(wo == (wi + 1) / 2);
  assert(ho == (hi + 1) / 2);

  for (int y = 0; y < ho; ++y) {
    const auto y_1 = 2 * y;
    const auto y_2 = std::min(2 * y + 1, hi - 1); // repeat bottom border

    for (int x = 0; x < wo; ++x) {
      const auto x_1 = 2 * x;
      const auto x_2 = std::min(2 * x + 1, wi - 1); // repeat right border

      out(y, x) = filter(std::array{in(y_1, x_1), in(y_1, x_2), in(y_2, x_1), in(y_2, x_2)});
    }
  }
}

template <typename InMatrix, typename OutMatrix, typename PullFilter>
void PushPull::inplacePull(const InMatrix &&in, int wi, int hi, OutMatrix &&out, int wo, int ho,
                           PullFilter &&filter) noexcept {
  assert(wi == (wo + 1) / 2);
  assert(hi == (ho + 1) / 2);

  for (int y = 0; y < ho; ++y) {
    auto y_1 = std::max(0, (y - 1) / 2);
    auto y_2 = std::min(hi - 1, (y + 1) / 2);
    if (y % 2 == 0) {
      std::swap(y_1, y_2);
    }

    for (int x = 0; x < wo; ++x) {
      auto x_1 = std::max(0, (x - 1) / 2);
      auto x_2 = std::min(wi - 1, (x + 1) / 2);
      if (x % 2 == 0) {
        std::swap(x_1, x_2);
      }
      const auto value = out(y, x);

      out(y, x) = filter(std::array{in(y_1, x_1), in(y_1, x_2), in(y_2, x_1), in(y_2, x_2)}, value);
    }
  }
}
} // namespace TMIV::Renderer
