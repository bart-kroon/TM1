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

#include "Engine.h"

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {
auto imagePositions(const CameraParameters &camera) -> Mat<Vec2f> {
  Mat<Vec2f> result;
  result.resize(camera.size.y(), camera.size.x());
  for (unsigned i = 0; i != result.height(); ++i) {
    for (unsigned j = 0; j != result.width(); ++j) {
      result(i, j) = {float(j) + 0.5f, float(i) + 0.5f};
    }
  }
  return result;
}

namespace {
template <ProjectionType type>
auto unprojectPoints(const Mat<Vec2f> &positions, const Mat<float> &depth,
                     const Engine<type> &engine) -> Mat<Vec3f> {
  Mat<Vec3f> points{positions.sizes()};
  assert(positions.sizes() == depth.sizes());
  transform(
      begin(positions), end(positions), begin(depth), begin(points),
      [=](Vec2f uv, float depth) { return engine.unprojectVertex(uv, depth); });
  return points;
}
} // namespace

auto unprojectPoints(const CameraParameters &camera,
                     const Mat<Vec2f> &positions, const Mat<float> &depth)
    -> Mat<Vec3f> {
  switch (camera.type) {
  case ProjectionType::ERP: {
    Engine<ProjectionType::ERP> engine{camera};
    return unprojectPoints(positions, depth, engine);
  }
  case ProjectionType::Perspective: {
    Engine<ProjectionType::Perspective> engine{camera};
    return unprojectPoints(positions, depth, engine);
  }
  default:
    abort();
  }
}

auto changeReferenceFrame(const CameraParameters &camera,
                          const CameraParameters &target,
                          const Mat<Vec3f> &points) -> Mat<Vec3f> {
  Mat<Vec3f> result(points.sizes());
  const auto R_t = affineParameters(camera, target);
  transform(begin(points), end(points), begin(result),
            [R = R_t.first, t = R_t.second](Vec3f x) { return R * x + t; });
  return result;
}

namespace {
template <ProjectionType TYPE>
auto projectPoints(const Mat<Vec3f> &points, const Engine<TYPE> &engine)
    -> pair<Mat<Vec2f>, Mat<float>> {
  Mat<Vec2f> positions{points.sizes()};
  Mat<float> depth{points.sizes()};

  auto i_positions = begin(positions);
  auto i_depth = begin(depth);

  for (auto point : points) {
    ImageVertexDescriptor v = engine.projectVertex({point, 0.f});
    *i_positions++ = v.position;
    *i_depth++ = v.depth;
  }

  return {positions, depth};
}
} // namespace

auto projectPoints(const CameraParameters &camera, const Mat<Vec3f> &points)
    -> pair<Mat<Vec2f>, Mat<float>> {
  switch (camera.type) {
  case ProjectionType::ERP: {
    Engine<ProjectionType::ERP> engine{camera};
    return projectPoints(points, engine);
  }
  case ProjectionType::Perspective: {
    Engine<ProjectionType::Perspective> engine{camera};
    return projectPoints(points, engine);
  }
  default:
    abort();
  }
}

auto reprojectPoints(const CameraParameters &camera,
                     const CameraParameters &target,
                     const Mat<Vec2f> &positions, const Mat<float> &depth)
    -> pair<Mat<Vec2f>, Mat<float>> {
  auto points = unprojectPoints(camera, positions, depth);
  points = changeReferenceFrame(camera, target, points);
  return projectPoints(target, points);
}

auto calculateRayAngles(const CameraParameters &camera,
                        const CameraParameters &target,
                        const Mat<Vec3f> &points) -> Mat<float> {
  Mat<float> result(points.sizes());
  const auto R_t = affineParameters(camera, target);
  transform(begin(points), end(points), begin(result),
            [t = R_t.second](Vec3f virtualRay) {
              return angle(virtualRay, virtualRay - t);
            });
  return result;
}
} // namespace TMIV::Renderer
