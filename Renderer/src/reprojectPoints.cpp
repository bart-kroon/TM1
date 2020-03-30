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

#include <TMIV/Renderer/reprojectPoints.h>

#include <TMIV/Common/Thread.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Renderer {
constexpr auto halfPixel = 0.5F;

auto imagePositions(const CameraIntrinsics &ci) -> Mat<Vec2f> {
  Mat<Vec2f> result;
  result.resize(ci.projectionPlaneSize().y(), ci.projectionPlaneSize().x());
  for (unsigned i = 0; i != result.height(); ++i) {
    for (unsigned j = 0; j != result.width(); ++j) {
      result(i, j) = {float(j) + halfPixel, float(i) + halfPixel};
    }
  }
  return result;
}

auto unprojectPoints(const CameraIntrinsics &ci, const Mat<Vec2f> &positions,
                     const Mat<float> &depth) -> Mat<Vec3f> {
  assert(positions.sizes() == depth.sizes());
  return ci.dispatch([&](auto camType) {
    Engine<camType.value> engine{ci};
    Mat<Vec3f> points{positions.sizes()};

    parallel_for(points.size(),
                 [&](size_t id) { points[id] = engine.unprojectVertex(positions[id], depth[id]); });

    return points;
  });
}

auto changeReferenceFrame(const CameraExtrinsics &source, const CameraExtrinsics &target,
                          const Mat<Vec3f> &points) -> Mat<Vec3f> {
  Mat<Vec3f> result(points.sizes());
  const auto R_t = AffineTransform{source, target};

  parallel_for(points.size(), [&](size_t id) { result[id] = R_t(points[id]); });
  return result;
}

auto projectPoints(const CameraIntrinsics &ci, const Mat<Vec3f> &points)
    -> pair<Mat<Vec2f>, Mat<float>> {
  return ci.dispatch([&](auto camType) {
    Engine<camType.value> engine{ci};

    Mat<Vec2f> positions{points.sizes()};
    Mat<float> depth{points.sizes()};

    parallel_for(points.size(), [&](size_t id) {
      ImageVertexDescriptor v = engine.projectVertex({points[id], 0.F});
      positions[id] = v.position;
      depth[id] = v.depth;
    });

    return pair{positions, depth};
  });
}

auto reprojectPoints(const ViewParams &source, const ViewParams &target,
                     const Mat<Vec2f> &positions, const Mat<float> &depth)
    -> pair<Mat<Vec2f>, Mat<float>> {
  auto points = unprojectPoints(source.ci, positions, depth);
  points = changeReferenceFrame(source.ce, target.ce, points);
  return projectPoints(target.ci, points);
}

auto calculateRayAngles(const CameraExtrinsics &source, const CameraExtrinsics &target,
                        const Mat<Vec3f> &points) -> Mat<float> {
  Mat<float> result(points.sizes());
  const auto t = AffineTransform(source, target).translation();
  transform(begin(points), end(points), begin(result),
            [t](Vec3f virtualRay) { return angle(virtualRay, virtualRay - t); });
  return result;
}

AffineTransform::AffineTransform(const CameraExtrinsics &source, const CameraExtrinsics &target) {
  const auto r1 = QuatD(source.rotation());
  const auto r2 = QuatD(target.rotation());
  const auto t1 = Vec3d(source.position());
  const auto t2 = Vec3d(target.position());

  const auto r = conj(r2) * r1;
  const auto t = rotate(t1 - t2, conj(r2));

  m_R = Mat3x3f(rotationMatrix(r));
  m_t = Vec3f(t);
}

auto AffineTransform::operator()(Vec3f x) const -> Vec3f { return m_R * x + m_t; }

auto unprojectVertex(Vec2f position, float depth, const CameraIntrinsics &ci) -> Vec3f {
  return ci.dispatch([&](auto camType) {
    Engine<camType> engine{ci};
    return engine.unprojectVertex(position, depth);
  });
}

auto projectVertex(const Vec3f &position, const CameraIntrinsics &ci) -> pair<Vec2f, float> {
  return ci.dispatch([&](auto camType) {
    Engine<camType> engine{ci};
    auto imageVertexDescriptor = engine.projectVertex(SceneVertexDescriptor{position, 0.F});
    return make_pair(imageVertexDescriptor.position, imageVertexDescriptor.depth);
  });
}

template <>
auto ProjectionHelper<CiCamType::equirectangular>::getAngularResolution() const -> float {
  const auto &ci = m_viewParams.ci;
  auto nbPixel = static_cast<float>(ci.projectionPlaneSize().x() * ci.projectionPlaneSize().y());
  float DT = ci.ci_erp_phi_max() - ci.ci_erp_phi_min();
  float DS = sin(ci.ci_erp_theta_max()) - sin(ci.ci_erp_theta_min());

  return nbPixel / (DS * DT);
}

template <> auto ProjectionHelper<CiCamType::perspective>::getAngularResolution() const -> float {
  const auto &ci = m_viewParams.ci;
  auto nbPixel = static_cast<float>(ci.projectionPlaneSize().x() * ci.projectionPlaneSize().y());
  const auto projectionFocalLength =
      (ci.ci_perspective_focal_hor() + ci.ci_perspective_focal_ver()) / 2.F;
  auto w = static_cast<float>(ci.projectionPlaneSize().x());
  auto h = static_cast<float>(ci.projectionPlaneSize().y());
  float omega = 4.F * atan(nbPixel / (2.F * projectionFocalLength *
                                      sqrt(4.F * sqr(projectionFocalLength) + (w * w + h * h))));

  return nbPixel / omega;
}

template <> auto ProjectionHelper<CiCamType::orthographic>::getAngularResolution() const -> float {
  const auto &ci = m_viewParams.ci;
  auto nbPixel = static_cast<float>(ci.projectionPlaneSize().x() * ci.projectionPlaneSize().y());
  return nbPixel / hemiSphere;
}

template <> auto ProjectionHelper<CiCamType::equirectangular>::getRadialRange() const -> Vec2f {
  return {1.F / m_viewParams.dq.dq_norm_disp_high(), 1.F / m_viewParams.dq.dq_norm_disp_low()};
}

template <> auto ProjectionHelper<CiCamType::perspective>::getRadialRange() const -> Vec2f {
  const auto &ci = m_viewParams.ci;
  float x = (static_cast<float>(m_viewParams.ci.projectionPlaneSize().x()) -
             ci.ci_perspective_center_hor()) /
            ci.ci_perspective_focal_hor();
  float y = (static_cast<float>(m_viewParams.ci.projectionPlaneSize().y()) -
             ci.ci_perspective_center_ver()) /
            ci.ci_perspective_focal_ver();

  return {1.F / m_viewParams.dq.dq_norm_disp_high(),
          norm(Vec3f{x, y, 1.F}) / m_viewParams.dq.dq_norm_disp_low()};
}

template <> auto ProjectionHelper<CiCamType::orthographic>::getRadialRange() const -> Vec2f {
  return {1.F / m_viewParams.dq.dq_norm_disp_high(), 1.F / m_viewParams.dq.dq_norm_disp_low()};
}
} // namespace TMIV::Renderer
