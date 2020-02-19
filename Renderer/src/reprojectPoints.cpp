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

#include <TMIV/Renderer/reprojectPoints.h>

#include <TMIV/Common/Thread.h>
#include <TMIV/Common/Transformation.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Renderer {
constexpr auto halfPixel = 0.5F;

auto imagePositions(const ViewParams &viewParams) -> Mat<Vec2f> {
  Mat<Vec2f> result;
  result.resize(viewParams.size.y(), viewParams.size.x());
  for (unsigned i = 0; i != result.height(); ++i) {
    for (unsigned j = 0; j != result.width(); ++j) {
      result(i, j) = {float(j) + halfPixel, float(i) + halfPixel};
    }
  }
  return result;
}

auto unprojectPoints(const ViewParams &viewParams, const Mat<Vec2f> &positions,
                     const Mat<float> &depth) -> Mat<Vec3f> {
  assert(positions.sizes() == depth.sizes());

  return visit(
      [&](const auto &projection) {
        Engine<decay_t<decltype(projection)>> engine{viewParams};
        Mat<Vec3f> points{positions.sizes()};

        parallel_for(points.size(), [&](size_t id) {
          points[id] = engine.unprojectVertex(positions[id], depth[id]);
        });

        return points;
      },
      viewParams.projection);
}

auto changeReferenceFrame(const ViewParams &viewParams, const ViewParams &target,
                          const Mat<Vec3f> &points) -> Mat<Vec3f> {
  Mat<Vec3f> result(points.sizes());
  const auto R_t = affineParameters(viewParams, target);

  parallel_for(points.size(), [&](size_t id) { result[id] = R_t.first * points[id] + R_t.second; });

  return result;
}

auto projectPoints(const ViewParams &viewParams, const Mat<Vec3f> &points)
    -> pair<Mat<Vec2f>, Mat<float>> {
  return visit(
      [&](const auto &projection) {
        Engine<decay_t<decltype(projection)>> engine{viewParams};

        Mat<Vec2f> positions{points.sizes()};
        Mat<float> depth{points.sizes()};

        parallel_for(points.size(), [&](size_t id) {
          ImageVertexDescriptor v = engine.projectVertex({points[id], 0.F});
          positions[id] = v.position;
          depth[id] = v.depth;
        });

        return pair{positions, depth};
      },
      viewParams.projection);
}

auto reprojectPoints(const ViewParams &viewParams, const ViewParams &target,
                     const Mat<Vec2f> &positions, const Mat<float> &depth)
    -> pair<Mat<Vec2f>, Mat<float>> {
  auto points = unprojectPoints(viewParams, positions, depth);
  points = changeReferenceFrame(viewParams, target, points);
  return projectPoints(target, points);
}

auto calculateRayAngles(const ViewParams &viewParams, const ViewParams &target,
                        const Mat<Vec3f> &points) -> Mat<float> {
  Mat<float> result(points.sizes());
  const auto R_t = affineParameters(viewParams, target);
  transform(begin(points), end(points), begin(result),
            [t = R_t.second](Vec3f virtualRay) { return angle(virtualRay, virtualRay - t); });
  return result;
}

auto affineParameters(const ViewParams &viewParams, const ViewParams &target)
    -> pair<Mat3x3f, Vec3f> {
  const auto R1 = viewParams.ce.rotationMatrix();
  const auto R2 = target.ce.rotationMatrix();
  const auto &t1 = viewParams.ce.position();
  const auto &t2 = target.ce.position();

  const auto R = transpose(R2) * R1;
  const auto t = transpose(R2) * (t1 - t2);
  return {R, t};
}

auto unprojectVertex(Vec2f position, float depth, const ViewParams &viewParams) -> Vec3f {
  return visit(
      [&](const auto &projection) {
        Engine<decay_t<decltype(projection)>> engine{viewParams};
        return engine.unprojectVertex(position, depth);
      },
      viewParams.projection);
}

auto projectVertex(const Common::Vec3f &position, const MivBitstream::ViewParams &viewParams)
    -> std::pair<Common::Vec2f, float> {
  return visit(
      [&](const auto &projection) {
        Engine<decay_t<decltype(projection)>> engine{viewParams};
        auto imageVertexDescriptor = engine.projectVertex(SceneVertexDescriptor{position, 0.F});
        return std::make_pair(imageVertexDescriptor.position, imageVertexDescriptor.depth);
      },
      viewParams.projection);
}

template <> auto ProjectionHelper<ErpParams>::getAngularResolution() const -> float {
  auto nbPixel = static_cast<float>(m_viewParams.size.x() * m_viewParams.size.y());
  const auto &erpParams = std::get<ErpParams>(m_viewParams.projection);

  float DT = radperdeg * (erpParams.phiRange[1] - erpParams.phiRange[0]);
  float DS =
      std::sin(radperdeg * erpParams.thetaRange[1]) - std::sin(radperdeg * erpParams.thetaRange[0]);

  return nbPixel / (DS * DT);
}

template <> auto ProjectionHelper<PerspectiveParams>::getAngularResolution() const -> float {
  auto nbPixel = static_cast<float>(m_viewParams.size.x() * m_viewParams.size.y());
  const auto &perspectiveParams = std::get<PerspectiveParams>(m_viewParams.projection);

  float projectionFocalLength = (perspectiveParams.focal.x() + perspectiveParams.focal.y()) / 2.F;
  auto w = static_cast<float>(m_viewParams.size.x());
  auto h = static_cast<float>(m_viewParams.size.y());

  float omega =
      4.F * std::atan(nbPixel / (2.F * projectionFocalLength *
                                 std::sqrt(4.F * sqr(projectionFocalLength) + (w * w + h * h))));

  return nbPixel / omega;
}

template <> auto ProjectionHelper<ErpParams>::getRadialRange() const -> Vec2f {
  return {1.F / m_viewParams.dq.dq_norm_disp_high(), 1.F / m_viewParams.dq.dq_norm_disp_low()};
}

template <> auto ProjectionHelper<PerspectiveParams>::getRadialRange() const -> Vec2f {
  const auto &perspectiveParams = std::get<PerspectiveParams>(m_viewParams.projection);

  float x = (static_cast<float>(m_viewParams.size.x()) - perspectiveParams.center.x()) /
            perspectiveParams.focal.x();
  float y = (static_cast<float>(m_viewParams.size.y()) - perspectiveParams.center.y()) /
            perspectiveParams.focal.y();

  return {1.F / m_viewParams.dq.dq_norm_disp_high(),
          norm(Vec3f{x, y, 1.F}) / m_viewParams.dq.dq_norm_disp_low()};
}
} // namespace TMIV::Renderer
