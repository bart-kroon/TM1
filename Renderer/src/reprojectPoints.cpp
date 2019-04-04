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

#include <TMIV/Renderer/reprojectPoints.h>

#include <algorithm>
#include <cassert>
#include <cmath>

#include <TMIV/Common/LinAlg.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {
namespace {
const float radperdeg = 0.01745329251994329576923690768489f;
const float degperrad = 57.295779513082320876798154814092f;
const float NaN = numeric_limits<float>::quiet_NaN();

Mat3x3f rotationMatrixFromRotationAroundX(float rx) {
  return Mat3x3f{1.f, 0.f, 0.f, 0.f, cos(rx), -sin(rx), 0.f, sin(rx), cos(rx)};
}

Mat3x3f rotationMatrixFromRotationAroundY(float ry) {
  return Mat3x3f{cos(ry), 0.f, sin(ry), 0.f, 1.f, 0.f, -sin(ry), 0.f, cos(ry)};
}

Mat3x3f rotationMatrixFromRotationAroundZ(float rz) {
  return Mat3x3f{cos(rz), -sin(rz), 0.f, sin(rz), cos(rz), 0.f, 0.f, 0.f, 1.f};
}

Mat3x3f EulerAnglesToRotationMatrix(Vec3f rotation) {
  return mult(mult(rotationMatrixFromRotationAroundZ(rotation[0]),
                   rotationMatrixFromRotationAroundY(rotation[1])),
              rotationMatrixFromRotationAroundX(rotation[2]));
}

template <ProjectionType TYPE> class Unprojector {};
template <ProjectionType TYPE> class Projector {};

template <> class Unprojector<ProjectionType::ERP> {
public:
  explicit Unprojector(const CameraParameters &camera)
      : m_phi0{radperdeg * camera.erpPhiRange[1]},
        m_theta0{radperdeg * camera.erpThetaRange[1]},
        m_dphi_du{-radperdeg * (camera.erpPhiRange[1] - camera.erpPhiRange[0]) /
                  camera.size.x()},
        m_dtheta_dv{-radperdeg *
                    (camera.erpThetaRange[1] - camera.erpThetaRange[0]) /
                    camera.size.y()} {};

  Vec3f operator()(Vec2f position, float depth) const {
    const float phi = m_phi0 + m_dphi_du * position.x();
    const float theta = m_theta0 + m_dtheta_dv * position.y();
    return depth *
           Vec3f{cos(theta) * cos(phi), cos(theta) * sin(phi), sin(theta)};
  }

private:
  const float m_phi0;
  const float m_theta0;
  const float m_dphi_du;
  const float m_dtheta_dv;
};

template <> class Projector<ProjectionType::ERP> {
public:
  explicit Projector(const CameraParameters &camera)
      : m_u0{camera.size.x() * camera.erpPhiRange[1] /
             (camera.erpPhiRange[1] - camera.erpPhiRange[0])},
        m_v0{camera.size.y() * camera.erpThetaRange[1] /
             (camera.erpThetaRange[1] - camera.erpThetaRange[0])},
        m_du_dphi{-degperrad * camera.size.x() /
                  (camera.erpPhiRange[1] - camera.erpPhiRange[0])},
        m_dv_dtheta{-degperrad * camera.size.y() /
                    (camera.erpThetaRange[1] - camera.erpThetaRange[0])} {}

  pair<Vec2f, float> operator()(Vec3f point) const {
    const auto radius = norm(point);
    const auto phi = atan2(point.y(), point.x());
    const auto theta = asin(point.z() / radius);
    const auto position =
        Vec2f{m_u0 + m_du_dphi * phi, m_v0 + m_dv_dtheta * theta};
    return {position, radius};
  }

private:
  const float m_u0;
  const float m_v0;
  const float m_du_dphi;
  const float m_dv_dtheta;
};

template <> class Unprojector<ProjectionType::Perspective> {
public:
  explicit Unprojector(const CameraParameters &camera)
      : m_f{camera.perspectiveFocal}, m_p{camera.perspectiveCenter} {}

  Vec3f operator()(Vec2f uv, float d) const {
    if (d > 0.f) {
      return Vec3f{d, -(d / m_f[0]) * (uv[0] - m_p[0]),
                   -(d / m_f[1]) * (uv[1] - m_p[1])};
    }
    return Vec3f{NaN, NaN, NaN};
  }

private:
  Vec2f m_f;
  Vec2f m_p;
};

template <> class Projector<ProjectionType::Perspective> {
public:
  explicit Projector(const CameraParameters &camera)
      : m_f{camera.perspectiveFocal}, m_p{camera.perspectiveCenter} {}

  pair<Vec2f, float> operator()(Vec3f xyz) const {
    if (xyz.x() > 0.f) {
      auto uv = Vec2f{-m_f[0] * xyz[1] / xyz[0] + m_p[0],
                      -m_f[1] * xyz[2] / xyz[0] + m_p[1]};
      return {uv, xyz.x()};
    }
    return {{NaN, NaN}, NaN};
  }

private:
  Vec2f m_f;
  Vec2f m_p;
};
} // namespace

template <ProjectionType TYPE>
Mat3f unprojectPoints(const Mat2f &positions, const Mat1f &depth,
                      Unprojector<TYPE> unprojector) {
  Mat3f points{positions.sizes()};
  assert(positions.sizes() == depth.sizes());
  transform(begin(positions), end(positions), begin(depth), begin(points),
            [=](Vec2f uv, float depth) { return unprojector(uv, depth); });
  return points;
}

template <ProjectionType TYPE>
pair<Mat2f, Mat1f> projectPoints(const Mat3f &points,
                                 Projector<TYPE> projector) {
  Mat2f positions{points.sizes()};
  Mat1f depth{points.sizes()};

  auto i_positions = begin(positions);
  auto i_depth = begin(depth);

  for (auto point : points) {
    auto uv_d = projector(point);
    *i_positions++ = move(uv_d.first);
    *i_depth++ = move(uv_d.second);
  }

  return {positions, depth};
}

Mat3f unprojectPoints(const CameraParameters &camera, const Mat2f &positions,
                      const Mat1f &depth) {
  switch (camera.type) {
  case ProjectionType::ERP:
    return unprojectPoints(positions, depth,
                           Unprojector<ProjectionType::ERP>{camera});
  case ProjectionType::Perspective:
    return unprojectPoints(positions, depth,
                           Unprojector<ProjectionType::Perspective>{camera});
  default:
    throw logic_error("Projection type unknown or not yet implemented");
  }
}

Mat3f changeReferenceFrame(const CameraParameters &fromCamera,
                           const CameraParameters &toCamera, Mat3f points) {
  const auto R1 = EulerAnglesToRotationMatrix(fromCamera.rotation);
  const auto R2 = EulerAnglesToRotationMatrix(toCamera.rotation);
  const auto &t1 = fromCamera.position;
  const auto &t2 = toCamera.position;

  const auto R = transpose(R2) * R1;
  const auto t = transpose(R2) * (t2 - t1);

  for (auto &point : points) {
    point = mult(R, point) + t;
  }
  return points;
}

std::pair<Mat2f, Mat1f> projectPoints(const CameraParameters &camera,
                                      const Mat3f &points) {
  switch (camera.type) {
  case ProjectionType::ERP:
    return projectPoints(points, Projector<ProjectionType::ERP>{camera});
  case ProjectionType::Perspective:
    return projectPoints(points,
                         Projector<ProjectionType::Perspective>{camera});
  default:
    throw logic_error("Projection type unknown or not yet implemented");
  }
}

std::pair<Mat2f, Mat1f> reprojectPoints(const CameraParameters &fromCamera,
                                        const CameraParameters &toCamera,
                                        const Mat2f &positions,
                                        const Mat1f &depth) {
  auto points = unprojectPoints(fromCamera, positions, depth);
  points = changeReferenceFrame(fromCamera, toCamera, points);
  return projectPoints(toCamera, points);
}
} // namespace TMIV::Renderer
