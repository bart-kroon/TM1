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

ProjectionHelper::List::List(const MivBitstream::ViewParamsList &viewParamsList) {
  for (const auto &viewParams : viewParamsList) {
    this->emplace_back(viewParams);
  }
}

ProjectionHelper::ProjectionHelper(const MivBitstream::ViewParams &viewParams)
    : m_viewParams{viewParams}, m_rotation{viewParams.ce.rotation()} {
  switch (viewParams.ci.ci_cam_type()) {
  case CiCamType::equirectangular:
    m_engine = std::make_unique<MetaEngine::Equirectangular>(viewParams.ci);
    break;
  case CiCamType::perspective:
    m_engine = std::make_unique<MetaEngine::Perspective>(viewParams.ci);
    break;
  case CiCamType::orthographic:
    m_engine = std::make_unique<MetaEngine::Orthographic>(viewParams.ci);
    break;
  default:
    abort();
  }
}

auto ProjectionHelper::getViewingDirection() const -> Common::Vec3f {
  return rotate(Common::Vec3f{1.F, 0.F, 0.F}, m_rotation);
}

auto ProjectionHelper::changeFrame(const Common::Vec3f &P) const -> Common::Vec3f {
  return rotate(P - m_viewParams.ce.position(), conj(m_rotation));
}

auto ProjectionHelper::doProjection(const Common::Vec3f &P) const
    -> std::pair<Common::Vec2f, float> {
  Common::Vec3f Q = changeFrame(P);
  auto imageVertexDescriptor = m_engine->projectVertex(SceneVertexDescriptor{Q, 0.F});
  return std::make_pair(imageVertexDescriptor.position, imageVertexDescriptor.depth);
}

auto ProjectionHelper::doUnprojection(const Common::Vec2f &p, float d) const -> Common::Vec3f {
  auto P = m_engine->unprojectVertex(p, d);
  return rotate(P, m_rotation) + m_viewParams.ce.position();
}

auto ProjectionHelper::isStrictlyInsideViewport(const Common::Vec2f &p) const -> bool {
  return ((0.5F <= p.x()) && (p.x() <= (m_viewParams.ci.projectionPlaneSize().x() - 0.5F))) &&
         ((0.5F <= p.y()) && (p.y() <= (m_viewParams.ci.projectionPlaneSize().y() - 0.5F)));
}

auto ProjectionHelper::isInsideViewport(const Common::Vec2f &p) const -> bool {
  return ((-0.5F <= p.x()) && (p.x() <= (m_viewParams.ci.projectionPlaneSize().x() + 0.5F))) &&
         ((-0.5F <= p.y()) && (p.y() <= (m_viewParams.ci.projectionPlaneSize().y() + 0.5F)));
}

auto ProjectionHelper::isValidDepth(float d) const -> bool {
  static constexpr auto far = 999.999F;
  return (TMIV::Renderer::isValidDepth(d) && (m_viewParams.dq.dq_norm_disp_low() <= (1.F / d)) &&
          (d < far));
}

auto ProjectionHelper::getAngularResolution() const -> float {
  const auto &ci = m_viewParams.ci;

  switch (ci.ci_cam_type()) {
  case CiCamType::equirectangular: {
    auto nbPixel = static_cast<float>(ci.projectionPlaneSize().x() * ci.projectionPlaneSize().y());
    float DT = ci.ci_erp_phi_max() - ci.ci_erp_phi_min();
    float DS = sin(ci.ci_erp_theta_max()) - sin(ci.ci_erp_theta_min());

    return nbPixel / (DS * DT);
  }
  case CiCamType::perspective: {
    auto nbPixel = static_cast<float>(ci.projectionPlaneSize().x() * ci.projectionPlaneSize().y());
    const auto projectionFocalLength =
        (ci.ci_perspective_focal_hor() + ci.ci_perspective_focal_ver()) / 2.F;
    auto w = static_cast<float>(ci.projectionPlaneSize().x());
    auto h = static_cast<float>(ci.projectionPlaneSize().y());
    float omega = 4.F * atan(nbPixel / (2.F * projectionFocalLength *
                                        sqrt(4.F * sqr(projectionFocalLength) + (w * w + h * h))));

    return nbPixel / omega;
  }
  case CiCamType::orthographic: {
    auto nbPixel = static_cast<float>(ci.projectionPlaneSize().x() * ci.projectionPlaneSize().y());
    return nbPixel / hemiSphere;
  }
  default:
    abort();
  }
}

auto ProjectionHelper::getDepthRange() const -> Common::Vec2f {
  return {1.F / m_viewParams.dq.dq_norm_disp_high(), 1.F / m_viewParams.dq.dq_norm_disp_low()};
}

auto ProjectionHelper::getRadialRange() const -> Vec2f {
  switch (m_viewParams.ci.ci_cam_type()) {
  case CiCamType::equirectangular: {
    return {1.F / m_viewParams.dq.dq_norm_disp_high(), 1.F / m_viewParams.dq.dq_norm_disp_low()};
  }
  case CiCamType::perspective: {
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
  case CiCamType::orthographic: {
    return {1.F / m_viewParams.dq.dq_norm_disp_high(), 1.F / m_viewParams.dq.dq_norm_disp_low()};
  }
  default:
    abort();
  }
}

auto ProjectionHelper::getPointCloud(unsigned N) const -> PointCloud {
  PointCloud pointCloud;

  float step = 1.F / static_cast<float>(N - 1U);
  auto depthRange = getDepthRange();

  float x = 0.F;

  for (unsigned i = 0U; i < N; i++) {
    float y = 0.F;

    float px = x * static_cast<float>(m_viewParams.ci.projectionPlaneSize().x());

    for (unsigned j = 0U; j < N; j++) {
      float d = depthRange.x();

      float py = y * static_cast<float>(m_viewParams.ci.projectionPlaneSize().y());

      for (unsigned k = 0U; k < N; k++) {
        pointCloud.emplace_back(doUnprojection({px, py}, d));

        d += step * (depthRange.y() - depthRange.x());
      }

      y += step;
    }

    x += step;
  }

  return pointCloud;
}

auto getPointCloudList(const ProjectionHelperList &sourceHelperList, unsigned N) -> PointCloudList {
  PointCloudList pointCloudList;

  for (const auto &helper : sourceHelperList) {
    pointCloudList.emplace_back(helper.getPointCloud(N));
  }

  return pointCloudList;
}

auto getOverlapping(const ProjectionHelperList &sourceHelperList,
                    const PointCloudList &pointCloudList, std::size_t firstId, std::size_t secondId)
    -> float {
  std::size_t N = 0;

  const ProjectionHelper &secondHelper = sourceHelperList[secondId];
  const PointCloud &firstPointCloud = pointCloudList[firstId];

  for (const auto &P : firstPointCloud) {
    auto p = secondHelper.doProjection(P);

    if (isValidDepth(p.second) && secondHelper.isInsideViewport(p.first)) {
      N++;
    }
  }

  return static_cast<float>(N) / static_cast<float>(firstPointCloud.size());
}

auto computeOverlappingMatrix(const ProjectionHelperList &sourceHelperList) -> Common::Mat<float> {
  auto pointCloudList = getPointCloudList(sourceHelperList, 16);
  std::size_t K = sourceHelperList.size();
  Common::Mat<float> overlappingMatrix({K, K});

  for (std::size_t i = 0; i < K; i++) {
    for (std::size_t j = 0; j < K; j++) {
      if (i != j) {
        overlappingMatrix(i, j) = getOverlapping(sourceHelperList, pointCloudList, i, j);
      } else {
        overlappingMatrix(i, j) = 1.F;
      }
    }
  }

  return overlappingMatrix;
}

} // namespace TMIV::Renderer
