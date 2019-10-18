/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#ifndef _TMIV_RENDERER_ENGINE_H_
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/Common.h>

#include <cassert>

namespace TMIV::Renderer {
template <> struct Engine<Metadata::PerspectiveParams> {
  const Metadata::ViewParams viewParams;
  const int icols;
  const int irows;
  const int ocols;
  const int orows;
  const int osize;
  const int numTriangles;
  const Common::Vec2f f;
  const Common::Vec2f p;

  explicit Engine(const Metadata::ViewParams &viewParams_)
      : viewParams{viewParams_},

        // Mesh structure
        icols{viewParams.size.x()}, irows{viewParams.size.y()}, ocols{viewParams.size.x() + 2},
        orows{viewParams.size.y() + 2}, osize{ocols * orows}, numTriangles{2 * (orows - 1) *
                                                                           (ocols - 1)},

        // Projection parameters
        f{viewParams.perspective().focal}, p{viewParams.perspective().center} {}

  // Unprojection equation
  auto unprojectVertex(Common::Vec2f uv, float depth) const -> Common::Vec3f {
    if (depth > 0.F) {
      return {depth, -(depth / f.x()) * (uv.x() - p.x()), -(depth / f.y()) * (uv.y() - p.y())};
    }
    return {Common::NaN, Common::NaN, Common::NaN};
  }

  // Projection equation
  auto projectVertex(const SceneVertexDescriptor &v) const -> ImageVertexDescriptor const {
    if (v.position.x() > 0.F) {
      auto uv = Common::Vec2f{-f.x() * v.position.y() / v.position.x() + p.x(),
                              -f.y() * v.position.z() / v.position.x() + p.y()};
      return {uv, v.position.x(), v.rayAngle};
    }
    return {{Common::NaN, Common::NaN}, Common::NaN, Common::NaN};
  }

  // Helper function to calculate the v-component of image coordinates at output
  // row i
  float vAt(int i) const {
    if (i == 0) {
      return 0.F; // top edge of frame
    }
    if (i == orows - 1) {
      return float(irows); // bottom edge of frame
    }
    return float(i) - 0.5F; // row middle
  }

  // Helper function to calculate the u-component of image coordinates at output
  // column j
  float uAt(int j) const {
    if (j == 0) {
      return 0.F; // left edge of frame
    }
    if (j == ocols - 1) {
      return float(icols); // right edge of frame
    }
    return float(j) - 0.5F; // column centre
  }

  // Helper function to fetch a value from a matrix based on the output
  // coordinate (i, j)
  template <class T> T fetch(int i, int j, const Common::Mat<T> &matrix) const {
    i = std::max(0, std::min(irows - 1, i - 1));
    j = std::max(0, std::min(icols - 1, j - 1));
    return matrix(i, j);
  }

  // Helper function to calculate the area of a triangle based on the output
  // coordinate (i, j)
  float triangleArea(int i, int j) const {
    return (j == 0 || j == ocols - 1 ? 0.25F : 0.5F) * (i == 0 || i == orows - 1 ? 0.5F : 1.F);
  }

  // Project mesh to target view
  template <typename... T>
  auto project(SceneVertexDescriptorList sceneVertices, TriangleDescriptorList triangles,
               std::tuple<std::vector<T>...> attributes) {
    ImageVertexDescriptorList imageVertices;
    imageVertices.reserve(sceneVertices.size());
    for (const SceneVertexDescriptor &v : sceneVertices) {
      imageVertices.push_back(projectVertex(v));
    }
    return std::tuple{move(imageVertices), triangles, attributes};
  }
};
} // namespace TMIV::Renderer
