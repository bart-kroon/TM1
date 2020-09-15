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

#include "PrunedMesh.h"

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/Renderer/reprojectPoints.h>

#include <cassert>

using namespace std;
using namespace TMIV::MivBitstream;
using namespace TMIV::Common;
using namespace TMIV::Renderer;

namespace TMIV::Pruner {
auto unprojectPrunedView(const TextureDepth16Frame &view, const ViewParams &viewParams,
                         const Mat<uint8_t> &mask)
    -> tuple<SceneVertexDescriptorList, TriangleDescriptorList, vector<Vec3f>> {
  return viewParams.ci.dispatch([&](auto camType) {
    tuple<SceneVertexDescriptorList, TriangleDescriptorList, vector<Vec3f>> mesh;
    auto &vertices = std::get<0>(mesh);
    auto &triangles = std::get<1>(mesh);
    auto &attributes = std::get<2>(mesh);

    Engine<camType> engine{viewParams.ci};
    const auto size = viewParams.ci.projectionPlaneSize();
    const auto numPixels = size.x() * size.y();

    const auto &Y = view.texture.getPlane(0);
    const auto &U = view.texture.getPlane(1);
    const auto &V = view.texture.getPlane(2);
    const auto &D = view.depth.getPlane(0);

    assert(vertices.empty());
    vertices.reserve(numPixels);
    assert(attributes.empty());
    attributes.reserve(numPixels);

    vector<int> key;
    key.reserve(vertices.size());

    const auto depthTransform = DepthTransform<16>{viewParams.dq};

    for (int y = 0; y < size.y(); ++y) {
      for (int x = 0; x < size.x(); ++x) {
        key.push_back(int(vertices.size()));
        const auto D_yx = D(y, x);

        if (mask(y, x) > 0) {
          const auto uv = Vec2f{static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F};
          const auto d = depthTransform.expandDepth(D_yx);
          vertices.push_back({engine.unprojectVertex(uv, d), NaN});
          attributes.emplace_back(Vec3f{expandValue<10U>(Y(y, x)),
                                        expandValue<10U>(U(y / 2, x / 2)),
                                        expandValue<10U>(V(y / 2, x / 2))});
        }
      }
    }

    if (vertices.capacity() > 2 * vertices.size()) {
      vertices.shrink_to_fit();
      attributes.shrink_to_fit();
    }

    assert(triangles.empty());
    const auto maxTriangles = 2 * vertices.size();
    triangles.reserve(maxTriangles);

    const auto considerTriangle = [&](Vec2i a, Vec2i b, Vec2i c) {
      if (mask(a.y(), a.x()) == 0 || mask(b.y(), b.x()) == 0 || mask(c.y(), c.x()) == 0) {
        return;
      }

      const auto ia = key[a.y() * size.x() + a.x()];
      const auto ib = key[b.y() * size.x() + b.x()];
      const auto ic = key[c.y() * size.x() + c.x()];
      triangles.push_back({{ia, ib, ic}, 0.5F});
    };

    for (int y = 1; y < size.y(); ++y) {
      for (int x = 1; x < size.x(); ++x) {
        considerTriangle({x - 1, y - 1}, {x, y - 1}, {x, y});
        considerTriangle({x - 1, y - 1}, {x, y}, {x - 1, y});
      }
    }

    return mesh;
  });
}

auto project(const SceneVertexDescriptorList &vertices, const ViewParams &source,
             const ViewParams &target) -> ImageVertexDescriptorList {
  return target.ci.dispatch([&](auto camType) {
    ImageVertexDescriptorList result;
    Engine<camType.value> engine{target.ci};
    const auto R_t = AffineTransform{source.ce, target.ce};
    result.reserve(result.size());
    transform(begin(vertices), end(vertices), back_inserter(result), [&](SceneVertexDescriptor v) {
      const auto p = R_t(v.position);
      return engine.projectVertex({p, angle(p, p - R_t.translation())});
    });
    return result;
  });
}

void weightedSphere(const CameraIntrinsics &ci, const ImageVertexDescriptorList &vertices,
                    TriangleDescriptorList &triangles) {
  if (ci.ci_cam_type() == CiCamType::equirectangular) {
    Engine<CiCamType::equirectangular> engine{ci};
    for (auto &triangle : triangles) {
      auto v = 0.F;
      for (auto index : triangle.indices) {
        v += vertices[index].position.y() / 3.F;
      }
      const auto theta = engine.theta0 + engine.dtheta_dv * v;
      triangle.area = 0.5F / cos(theta);
    }
  }
}
} // namespace TMIV::Pruner
