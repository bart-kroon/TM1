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
#include <cmath>

namespace TMIV::Renderer {
template <> struct Engine<Metadata::ErpParams> {
  const float phi0;
  const float theta0;
  const float dphi_du;
  const float dtheta_dv;
  const float u0;
  const float v0;
  const float du_dphi;
  const float dv_dtheta;

  explicit Engine(const Metadata::ViewParams &viewParams)
      : // Precomputed values used in te unprojection equation
        phi0{Common::radperdeg * viewParams.erp().phiRange[1]},
        theta0{Common::radperdeg * viewParams.erp().thetaRange[1]},
        dphi_du{-Common::radperdeg * (viewParams.erp().phiRange[1] - viewParams.erp().phiRange[0]) /
                viewParams.size.x()},
        dtheta_dv{-Common::radperdeg *
                  (viewParams.erp().thetaRange[1] - viewParams.erp().thetaRange[0]) /
                  viewParams.size.y()},

        // Precomputed values used in the projection equation
        u0{viewParams.size.x() * viewParams.erp().phiRange[1] /
           (viewParams.erp().phiRange[1] - viewParams.erp().phiRange[0])},
        v0{viewParams.size.y() * viewParams.erp().thetaRange[1] /
           (viewParams.erp().thetaRange[1] - viewParams.erp().thetaRange[0])},
        du_dphi{-Common::degperrad * viewParams.size.x() /
                (viewParams.erp().phiRange[1] - viewParams.erp().phiRange[0])},
        dv_dtheta{-Common::degperrad * viewParams.size.y() /
                  (viewParams.erp().thetaRange[1] - viewParams.erp().thetaRange[0])} {}

  // Unprojection equation
  auto unprojectVertex(Common::Vec2f uv, float depth) const -> Common::Vec3f {
    using std::cos;
    using std::sin;
    const float phi = phi0 + dphi_du * uv.x();
    const float theta = theta0 + dtheta_dv * uv.y();
    return depth * Common::Vec3f{cos(theta) * cos(phi), cos(theta) * sin(phi), sin(theta)};
  }

  // Projection equation
  auto projectVertex(const SceneVertexDescriptor &v) const -> ImageVertexDescriptor const {
    using std::asin;
    using std::atan2;
    const auto radius = norm(v.position);
    const auto phi = atan2(v.position.y(), v.position.x());
    const auto theta = asin(v.position.z() / radius);
    const auto position = Common::Vec2f{u0 + du_dphi * phi, v0 + dv_dtheta * theta};
    return {position, radius, v.rayAngle};
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

    // Weighted sphere compensation of stretching
    for (auto &triangle : triangles) {
      auto v = 0.F;
      for (auto index : triangle.indices) {
        v += imageVertices[index].position.y() / 3.F;
      }
      const auto theta = theta0 + dtheta_dv * v;
      triangle.area /= cos(theta);
    }

    return std::tuple{move(imageVertices), triangles, attributes};
  }
};
} // namespace TMIV::Renderer