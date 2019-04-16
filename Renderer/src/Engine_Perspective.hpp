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

#ifndef _TMIV_RENDERER_ENGINE_H_
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/Common.h>
#include <cassert>

namespace TMIV::Renderer {
template <> struct Engine<Metadata::ProjectionType::Perspective> {
  const Metadata::CameraParameters camera;
  const int icols;
  const int irows;
  const int ocols;
  const int orows;
  const int osize;
  const int numTriangles;
  const Common::Vec2f f;
  const Common::Vec2f p;

  Engine(const Metadata::CameraParameters &camera_)
      : camera{camera_},

        // Mesh structure
        icols{camera.size.x()}, irows{camera.size.y()},
        ocols{camera.size.x() + 2}, orows{camera.size.y() + 2},
        osize{ocols * orows}, numTriangles{2 * (orows - 1) * (ocols - 1)},

        // Projection parameters
        f{camera.perspectiveFocal}, p{camera.perspectiveCenter} {}

  // Unprojection equation
  auto unprojectVertex(Common::Vec2f uv, float depth) const -> Common::Vec3f {
    if (depth > 0.f) {
      return {depth, -(depth / f.x()) * (uv.x() - p.x()),
              -(depth / f.y()) * (uv.y() - p.y())};
    }
    return {Common::NaN, Common::NaN, Common::NaN};
  }

  // Projection equation
  auto projectVertex(const SceneVertexDescriptor &v) const
      -> ImageVertexDescriptor const {
    if (v.position.x() > 0.f) {
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
      return 0.f; // top edge of frame
    }
    if (i == orows - 1) {
      return float(irows); // bottom edge of frame
    }
    return float(i) - 0.5f; // row middle
  }

  // Helper function to calculate the u-component of image coordinates at output
  // column j
  float uAt(int j) const {
    if (j == 0) {
      return 0.f; // left edge of frame
    }
    if (j == ocols - 1) {
      return float(icols); // right edge of frame
    }
    return float(j) - 0.5f; // column centre
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
    return (j == 0 || j == ocols - 1 ? 0.25f : 0.5f) *
           (i == 0 || i == orows - 1 ? 0.5f : 1.f);
  }

  // List of 3-D vertices in the reference frame of the target camera
  auto
  makeSceneVertexDescriptorList(const Common::Mat<float> &depth,
                                const Metadata::CameraParameters &target) const
      -> SceneVertexDescriptorList {
    SceneVertexDescriptorList result;
    result.reserve(osize);
    const auto R_t = affineParameters(camera, target);
    for (int i = 0; i < orows; ++i) {
      for (int j = 0; j < ocols; ++j) {
        const auto u = uAt(j);
        const auto v = vAt(i);
        const auto d = fetch(i, j, depth);
        const auto xyz = R_t.first * unprojectVertex({u, v}, d) + R_t.second;
        const auto rayAngle = angle(xyz, xyz - R_t.second);
        result.push_back({xyz, rayAngle});
      }
    }
    assert(int(result.size()) == osize);
    return result;
  }

  // List of triangles with indices into the vertex lists
  auto makeTriangleDescriptorList() const -> TriangleDescriptorList {
    TriangleDescriptorList result;
    result.reserve(numTriangles);
    for (int i = 1; i < orows; ++i) {
      for (int j = 1; j < ocols; ++j) {
        const int br = i * ocols + j;
        const int tr = br - ocols;
        const int bl = br - 1;
        const int tl = tr - 1;
        const float area = triangleArea(i, j);
        result.push_back({{tl, tr, br}, area});
        result.push_back({{tl, br, bl}, area});
      }
    }
    assert(int(result.size()) == numTriangles);
    return result;
  }

  // List of vertex attributes in matching order
  template <class T>
  auto makeVertexAttributeList(const Common::Mat<T> &matrix) const
      -> std::vector<T> {
    std::vector<T> result;
    result.reserve(osize);
    for (int i = 0; i < orows; ++i) {
      for (int j = 0; j < ocols; ++j) {
        result.push_back(fetch(i, j, matrix));
      }
    }
    assert(int(result.size()) == osize);
    return result;
  }

  // Project mesh to target view
  //
  // TODO: Cull triangles
  template <typename... T>
  auto project(SceneVertexDescriptorList sceneVertices,
               TriangleDescriptorList triangles,
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
