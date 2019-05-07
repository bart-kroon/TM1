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
#include <cmath>

namespace TMIV::Renderer {
template <> struct Engine<Metadata::ProjectionType::ERP> {
  const Metadata::CameraParameters camera;
  const bool northPole;
  const bool southPole;
  const bool wraps;
  const int icols;
  const int irows;
  const int ocols;
  const int orows;
  const int osize;
  const int numTriangles;
  const float phi0;
  const float theta0;
  const float dphi_du;
  const float dtheta_dv;
  const float u0;
  const float v0;
  const float du_dphi;
  const float dv_dtheta;

  explicit Engine(const Metadata::CameraParameters &camera_)
      : camera{camera_},

        // Projection sub-type
        northPole{camera.erpThetaRange[1] == 90.f},
        southPole{camera.erpThetaRange[0] == -90.f},
        wraps{camera.erpPhiRange[0] == -180.f &&
              camera.erpPhiRange[1] == 180.f},

        // Mesh structure
        icols{camera.size.x()}, irows{camera.size.y()}, ocols{camera.size.x() +
                                                              2 - int(wraps)},
        orows{camera.size.y() + 2 - int(northPole) - int(southPole)},
        osize{ocols * orows + int(southPole) + int(northPole)},
        numTriangles{2 * (orows - 1) * (ocols - 1) +
                     (northPole ? ocols - 1 : 0) + (southPole ? ocols - 1 : 0)},

        // Precomputed values used in te unprojection equation
        phi0{Common::radperdeg * camera.erpPhiRange[1]},
        theta0{Common::radperdeg * camera.erpThetaRange[1]},
        dphi_du{-Common::radperdeg *
                (camera.erpPhiRange[1] - camera.erpPhiRange[0]) /
                camera.size.x()},
        dtheta_dv{-Common::radperdeg *
                  (camera.erpThetaRange[1] - camera.erpThetaRange[0]) /
                  camera.size.y()},

        // Precomputed values used in the projection equation
        u0{camera.size.x() * camera.erpPhiRange[1] /
           (camera.erpPhiRange[1] - camera.erpPhiRange[0])},
        v0{camera.size.y() * camera.erpThetaRange[1] /
           (camera.erpThetaRange[1] - camera.erpThetaRange[0])},
        du_dphi{-Common::degperrad * camera.size.x() /
                (camera.erpPhiRange[1] - camera.erpPhiRange[0])},
        dv_dtheta{-Common::degperrad * camera.size.y() /
                  (camera.erpThetaRange[1] - camera.erpThetaRange[0])} {}

  // Unprojection equation
  auto unprojectVertex(Common::Vec2f uv, float depth) const -> Common::Vec3f {
    const float phi = phi0 + dphi_du * uv.x();
    const float theta = theta0 + dtheta_dv * uv.y();
    return depth * Common::Vec3f{std::cos(theta) * std::cos(phi),
                                 std::cos(theta) * std::sin(phi),
                                 std::sin(theta)};
  }

  // Projection equation
  auto projectVertex(const SceneVertexDescriptor &v) const
      -> ImageVertexDescriptor const {
    const auto radius = norm(v.position);
    const auto phi = std::atan2(v.position.y(), v.position.x());
    const auto theta = std::asin(v.position.z() / radius);
    const auto position =
        Common::Vec2f{u0 + du_dphi * phi, v0 + dv_dtheta * theta};
    return {position, radius, v.rayAngle};
  }

  // Helper function to calculate the v-component of image coordinates at output
  // row i
  float vAt(int i) const {
    if (!northPole && i == 0) {
      return 0.f; // top edge of frame
    }
    if (!southPole && i == orows - 1) {
      return float(irows); // bottom edge of frame
    }
    return float(i) + int(northPole) - 0.5f; // row middle
  }

  // Helper function to calculate the u-component of image coordinates at output
  // column j
  float uAt(int j) const {
    if (!wraps && j == 0) {
      return 0.f; // left edge of frame
    }
    if (!wraps && j == ocols - 1) {
      return float(icols); // right edge of frame
    }
    return float(j) + int(wraps) - 0.5f; // column centre
  }

  // Helper function to fetch a value from a matrix based on the output
  // coordinate (i, j)
  template <class T> T fetch(int i, int j, const Common::Mat<T> &matrix) const {
    i = std::max(0, std::min(irows - 1, i + int(northPole) - 1));
    if (wraps) {
      j = j < icols ? j : 0;
    } else {
      j = std::max(0, std::min(icols - 1, j - 1));
    }
    return matrix(i, j);
  }

  // Helper function to average the value over an entire row (used for the
  // poles)
  template <class T>
  auto averageRow(const Common::Mat<T> &matrix, int row) const {
    auto sum = 0. * T();
    for (int column = 0; column < icols; ++column) {
      auto value = matrix(row, column);
      sum = sum + value;
    }
    if constexpr (std::is_arithmetic_v<T>) {
      return T(sum / icols);
    } else {
      T result;
      using V = typename T::value_type;
      std::transform(std::begin(sum), std::end(sum), std::begin(result),
                     [this](auto x) { return V(x / icols); });
      return result;
    }
  }

  // Helper function to calculate the area of a triangle based on the output
  // coordinate (i, j)
  float triangleArea(int i, int j) const {
    return (!wraps && (j == 0 || j == ocols - 1) ? 0.25f : 0.5f) *
           ((!northPole && i == 0) || (!southPole && i == orows - 1) ? 0.5f
                                                                     : 1.f);
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
    if (northPole) {
      const auto d = averageRow(depth, 0);
      const auto xyz = R_t.first * Common::Vec3f{0.f, 0.f, d} + R_t.second;
      const auto rayAngle = angle(xyz, xyz - R_t.second);
      result.push_back({xyz, rayAngle});
    }
    if (southPole) {
      const auto d = averageRow(depth, irows - 1);
      const auto xyz = R_t.first * Common::Vec3f{0.f, 0.f, -d} + R_t.second;
      const auto rayAngle = angle(xyz, xyz - R_t.second);
      result.push_back({xyz, rayAngle});
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
    if (northPole) {
      for (int j = 1; j < ocols; ++j) {
        const int t = osize - 2;
        const int br = j;
        const int bl = br - 1;
        const float area = !wraps && (j == 0 || j == ocols - 1) ? 0.25f : 0.5f;
        result.push_back({{t, br, bl}, area});
      }
    }
    if (southPole) {
      for (int j = 1; j < ocols; ++j) {
        const int b = osize - 1;
        const int tr = (orows - 1) * ocols + j;
        const int tl = tr - 1;
        const float area = !wraps && (j == 0 || j == ocols - 1) ? 0.25f : 0.5f;
        result.push_back({{tl, tr, b}, area});
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
    if (northPole) {
      result.push_back(averageRow(matrix, 0));
    }
    if (southPole) {
      result.push_back(averageRow(matrix, irows - 1));
    }
    assert(int(result.size()) == osize);
    return result;
  }

  // Project mesh to target view
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
