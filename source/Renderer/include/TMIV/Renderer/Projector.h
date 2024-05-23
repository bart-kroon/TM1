/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#ifndef TMIV_RENDERER_PROJECTOR_H
#define TMIV_RENDERER_PROJECTOR_H

#include <TMIV/Common/LinAlg.h>
#include <TMIV/MivBitstream/ViewParamsList.h>

#include <memory>

namespace TMIV::Renderer {
struct SceneVertexDescriptor {
  Common::Vec3f position; // m, scene point in target reference frame
  float rayAngle{};       // rad, ray angle from: cos a = <v, w>/|v||w|
};

struct TriangleDescriptor {
  std::array<int32_t, 3> indices; // indices into vertex lists
  float area;                     // px�, area before unprojection
};

struct ImageVertexDescriptor {
  Common::Vec2f position; // px, position in image (x right, y down)
  float depth{};          // m, depth as defined in the target projection
  float rayAngle{};       // rad, ray angle from: cos a = <v, w>/|v||w|
};

// The projector is the part that is specalized per projection type (static dispatch)
template <MivBitstream::CiCamType camType> class Projector {};

// Specialization for equirectangular projection
template <> class Projector<MivBitstream::CiCamType::equirectangular> {
public:
  explicit Projector(const MivBitstream::CameraIntrinsics &ci)
      : Projector{Common::deg2rad<double>(ci.ci_erp_phi_min()),
                  Common::deg2rad<double>(ci.ci_erp_phi_max()),
                  Common::deg2rad<double>(ci.ci_erp_theta_min()),
                  Common::deg2rad<double>(ci.ci_erp_theta_max()), ci.projectionPlaneSizeF()} {}

  // Unprojection equation
  [[nodiscard]] auto unprojectVertex(Common::Vec2f uv, float depth) const -> Common::Vec3f {
    using std::cos;
    using std::sin;
    const float phi = phi0 + dphi_du * uv.x();
    const float theta = theta0 + dtheta_dv * uv.y();
    return depth * Common::Vec3f{cos(theta) * cos(phi), cos(theta) * sin(phi), sin(theta)};
  }

  // Projection equation
  [[nodiscard]] auto projectVertex(const SceneVertexDescriptor &v) const -> ImageVertexDescriptor {
    using std::atan2;
    const auto radius = norm(v.position);
    const auto phi = atan2(v.position.y(), v.position.x());
    const auto theta = atan2(v.position.z(), std::hypot(v.position.x(), v.position.y()));
    const auto position = Common::Vec2f{u0 + du_dphi * phi, v0 + dv_dtheta * theta};
    return {position, radius, v.rayAngle};
  }

  [[nodiscard]] auto weightedSphere(float v) const { return std::cos(theta0 + dtheta_dv * v); }

private:
  explicit Projector(double phiMin, double phiMax, double thetaMin, double thetaMax,
                     Common::Vec2f projectionPlaneSize)
      : phi0{static_cast<float>(phiMax)}
      , theta0{static_cast<float>(thetaMax)}
      , dphi_du{static_cast<float>((phiMin - phiMax) / projectionPlaneSize.x())}
      , dtheta_dv{static_cast<float>((thetaMin - thetaMax) / projectionPlaneSize.y())}
      , u0{static_cast<float>(projectionPlaneSize.x() * phiMax / (phiMax - phiMin))}
      , v0{static_cast<float>(projectionPlaneSize.y() * thetaMax / (thetaMax - thetaMin))}
      , du_dphi{static_cast<float>(-projectionPlaneSize.x() / (phiMax - phiMin))}
      , dv_dtheta{static_cast<float>(-projectionPlaneSize.y() / (thetaMax - thetaMin))} {}

  float phi0;
  float theta0;
  float dphi_du;
  float dtheta_dv;
  float u0;
  float v0;
  float du_dphi;
  float dv_dtheta;
};

// Specialization for perspective projection
template <> class Projector<MivBitstream::CiCamType::perspective> {
public:
  explicit Projector(const MivBitstream::CameraIntrinsics &ci)
      : f_x{ci.ci_perspective_focal_hor()}
      , f_y{ci.ci_perspective_focal_ver()}
      , c_x{ci.ci_perspective_center_hor()}
      , c_y{ci.ci_perspective_center_ver()} {}

  // Unprojection equation
  [[nodiscard]] auto unprojectVertex(Common::Vec2f uv, float depth) const -> Common::Vec3f {
    if (depth > 0.F) {
      return {depth, -(depth / f_x) * (uv.x() - c_x), -(depth / f_y) * (uv.y() - c_y)};
    }
    return {NAN, NAN, NAN};
  }

  // Projection equation
  [[nodiscard]] auto projectVertex(const SceneVertexDescriptor &v) const -> ImageVertexDescriptor {
    if (v.position.x() > 0.F) {
      auto uv = Common::Vec2f{-f_x * v.position.y() / v.position.x() + c_x,
                              -f_y * v.position.z() / v.position.x() + c_y};
      return {uv, v.position.x(), v.rayAngle};
    }
    return {{NAN, NAN}, NAN, NAN};
  }

private:
  float f_x;
  float f_y;
  float c_x;
  float c_y;
};

// Specialization for orthographic projection
template <> class Projector<MivBitstream::CiCamType::orthographic> {
public:
  explicit Projector(const MivBitstream::CameraIntrinsics &ci)
      : ow{ci.ci_ortho_width()}
      , oh{ci.ci_ortho_height()}
      , ppw{static_cast<float>(ci.ci_projection_plane_width_minus1() + 1)}
      , pph{static_cast<float>(ci.ci_projection_plane_height_minus1() + 1)} {}

  // Unprojection equation
  [[nodiscard]] auto unprojectVertex(Common::Vec2f uv, float depth) const -> Common::Vec3f {
    return {depth, ow * (uv.x() / ppw - 0.5F), oh * (uv.y() / pph - 0.5F)};
  }

  // Projection equation
  [[nodiscard]] auto projectVertex(const SceneVertexDescriptor &v) const -> ImageVertexDescriptor {
    return {Common::Vec2f{ppw * (0.5F + v.position.y() / ow), pph * (0.5F + v.position.z() / oh)},
            v.position.x(), v.rayAngle};
  }

private:
  float ow;
  float oh;
  float ppw;
  float pph;
};

// Polymorphic projector (dynamic dispatch)
class IProjector {
public:
  IProjector() = default;
  IProjector(const IProjector &) = default;
  IProjector(IProjector &&) = default;
  auto operator=(IProjector &&) -> IProjector & = default;
  auto operator=(const IProjector &) -> IProjector & = default;
  virtual ~IProjector() = default;

  [[nodiscard]] virtual auto unprojectVertex(Common::Vec2f uv, float depth) const
      -> Common::Vec3f = 0;
  [[nodiscard]] virtual auto projectVertex(const SceneVertexDescriptor &v) const
      -> ImageVertexDescriptor = 0;
};

// Create polymorphic projector
[[nodiscard]] auto makeProjector(const MivBitstream::CameraIntrinsics &ci)
    -> std::unique_ptr<IProjector>;
} // namespace TMIV::Renderer

#endif
