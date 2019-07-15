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

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {
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
  return rotationMatrixFromRotationAroundZ(radperdeg * rotation[0]) *
         rotationMatrixFromRotationAroundY(radperdeg * rotation[1]) *
         rotationMatrixFromRotationAroundX(radperdeg * rotation[2]);
}

auto affineParameters(const CameraParameters &camera,
                      const CameraParameters &target) -> pair<Mat3x3f, Vec3f> {
  const auto R1 = EulerAnglesToRotationMatrix(camera.rotation);
  const auto R2 = EulerAnglesToRotationMatrix(target.rotation);
  const auto &t1 = camera.position;
  const auto &t2 = target.position;

  const auto R = transpose(R2) * R1;
  const auto t = transpose(R2) * (t1 - t2);
  return {R, t};
}

auto unprojectVertex(Common::Vec2f position, float depth,
                     const Metadata::CameraParameters &camera)
    -> Common::Vec3f {
  switch (camera.type) {
  case Metadata::ProjectionType::ERP: {
    Engine<Metadata::ProjectionType::ERP> engine{camera};
    return engine.unprojectVertex(position, depth);
  }
  case Metadata::ProjectionType::Perspective: {
    Engine<Metadata::ProjectionType::Perspective> engine{camera};
    return engine.unprojectVertex(position, depth);
  }
  default:
    abort();
  }
}
} // namespace TMIV::Renderer
