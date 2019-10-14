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

#include "Engine.h"
#include <TMIV/Common/Thread.h>
#include <TMIV/Renderer/reprojectPoints.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {
constexpr auto halfPixel = 0.5F;

auto imagePositions(const ViewParams &camera) -> Mat<Vec2f> {
  Mat<Vec2f> result;
  result.resize(camera.size.y(), camera.size.x());
  for (unsigned i = 0; i != result.height(); ++i) {
    for (unsigned j = 0; j != result.width(); ++j) {
      result(i, j) = {float(j) + halfPixel, float(i) + halfPixel};
    }
  }
  return result;
}

auto unprojectPoints(const ViewParams &camera, const Mat<Vec2f> &positions, const Mat<float> &depth)
    -> Mat<Vec3f> {
  assert(positions.sizes() == depth.sizes());

  return visit(
      [&](const auto &projection) {
        Engine<decay_t<decltype(projection)>> engine{camera};
        Mat<Vec3f> points{positions.sizes()};

        parallel_for(points.size(), [&](size_t id) {
          points[id] = engine.unprojectVertex(positions[id], depth[id]);
        });

        return points;
      },
      camera.projection);
}

auto changeReferenceFrame(const ViewParams &camera, const ViewParams &target,
                          const Mat<Vec3f> &points) -> Mat<Vec3f> {
  Mat<Vec3f> result(points.sizes());
  const auto R_t = affineParameters(camera, target);

  parallel_for(points.size(), [&](size_t id) { result[id] = R_t.first * points[id] + R_t.second; });

  return result;
}

auto projectPoints(const ViewParams &camera, const Mat<Vec3f> &points)
    -> pair<Mat<Vec2f>, Mat<float>> {
  return visit(
      [&](const auto &projection) {
        Engine<decay_t<decltype(projection)>> engine{camera};

        Mat<Vec2f> positions{points.sizes()};
        Mat<float> depth{points.sizes()};

        parallel_for(points.size(), [&](size_t id) {
          ImageVertexDescriptor v = engine.projectVertex({points[id], 0.F});
          positions[id] = v.position;
          depth[id] = v.depth;
        });

        return pair{positions, depth};
      },
      camera.projection);
}

auto reprojectPoints(const ViewParams &camera, const ViewParams &target,
                     const Mat<Vec2f> &positions, const Mat<float> &depth)
    -> pair<Mat<Vec2f>, Mat<float>> {
  auto points = unprojectPoints(camera, positions, depth);
  points = changeReferenceFrame(camera, target, points);
  return projectPoints(target, points);
}

auto calculateRayAngles(const ViewParams &camera, const ViewParams &target,
                        const Mat<Vec3f> &points) -> Mat<float> {
  Mat<float> result(points.sizes());
  const auto R_t = affineParameters(camera, target);
  transform(begin(points), end(points), begin(result),
            [t = R_t.second](Vec3f virtualRay) { return angle(virtualRay, virtualRay - t); });
  return result;
}
} // namespace TMIV::Renderer
