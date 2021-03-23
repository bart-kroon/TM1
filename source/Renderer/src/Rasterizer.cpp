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

#include <TMIV/Renderer/Rasterizer.h>

namespace TMIV::Renderer::detail {
auto determineTriangleBoundingBoxAndArea(int rows, int cols,
                                         const std::array<fixed_point::Vec2fp, 3> &uv) noexcept
    -> std::optional<TriangleInfo> {
  auto info = TriangleInfo{};

  using fixed_point::fpceil;
  using fixed_point::fpfloor;

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

auto calculateBarycentricCoordinate(int u, int v, const TriangleInfo &info,
                                    const std::array<fixed_point::Vec2fp, 3> &uv)
    -> std::optional<std::array<float, 3>> {
  using fixed_point::fixed;
  using fixed_point::half;
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
} // namespace TMIV::Renderer::detail
