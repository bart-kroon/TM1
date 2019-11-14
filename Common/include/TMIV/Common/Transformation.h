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

#ifndef _TMIV_COMMON_TRANSFORMATION_H_
#define _TMIV_COMMON_TRANSFORMATION_H_

#include "Common.h"
#include "LinAlg.h"

namespace TMIV::Common {

struct EulerAngles {
  Common::Vec3f value;
  explicit EulerAngles(const Common::Vec3f &eulerAngles) : value(eulerAngles) {}
};

auto rotationMatrixFromRotationAroundX(float rx) -> Common::Mat3x3f {
  using std::cos;
  using std::sin;
  return Common::Mat3x3f{1.F, 0.F, 0.F, 0.F, cos(rx), -sin(rx), 0.F, sin(rx), cos(rx)};
}

auto rotationMatrixFromRotationAroundY(float ry) -> Common::Mat3x3f {
  using std::cos;
  using std::sin;
  return Common::Mat3x3f{cos(ry), 0.F, sin(ry), 0.F, 1.F, 0.F, -sin(ry), 0.F, cos(ry)};
}

auto rotationMatrixFromRotationAroundZ(float rz) -> Common::Mat3x3f {
  using std::cos;
  using std::sin;
  return Mat3x3f{cos(rz), -sin(rz), 0.F, sin(rz), cos(rz), 0.F, 0.F, 0.F, 1.F};
}

auto EulerAnglesToRotationMatrix(Common::EulerAngles rotation) -> Common::Mat3x3f {
  return rotationMatrixFromRotationAroundZ(Common::radperdeg * rotation.value[0]) *
         rotationMatrixFromRotationAroundY(Common::radperdeg * rotation.value[1]) *
         rotationMatrixFromRotationAroundX(Common::radperdeg * rotation.value[2]);
}

} // namespace TMIV::Common

#endif
