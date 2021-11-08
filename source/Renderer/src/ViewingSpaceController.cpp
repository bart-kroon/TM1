/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#include <TMIV/Renderer/ViewingSpaceController.h>
#include <TMIV/ViewingSpace/ViewingSpaceEvaluator.h>

#include <cmath>

#include <iostream>

namespace TMIV::Renderer {
namespace {
auto computeIndex(const MivBitstream::ViewParams &metadata, const MivBitstream::ViewingSpace &vs)
    -> float {
  TMIV::ViewingSpace::ViewingParams vp;
  vp.viewPosition = metadata.pose.position;
  vp.viewRotation = Common::QuatF{Common::floatCast, metadata.pose.orientation};

  auto index = TMIV::ViewingSpace::ViewingSpaceEvaluator::computeInclusion(vs, vp);
  std::cout << "viewing space inclusion index: " << index << std::endl;

  return index;
}

void inplaceFading_impl(Common::RendererFrame &yuvd, const MivBitstream::ViewParams & /* unused */,
                        float index) {
  // TO DO
  auto &Y = yuvd.texture.getPlane(0);
  auto &U = yuvd.texture.getPlane(1);
  auto &V = yuvd.texture.getPlane(2);
  const auto maxValueF = static_cast<float>(yuvd.texture.maxValue());

  const auto width_Y = static_cast<int32_t>(Y.width());
  const auto height_Y = static_cast<int32_t>(Y.height());

  float weight = index; // for test:just a recopy of the index ==> mapping might be changed
  float R = 0.F;
  float G = 0.F;
  float B = 0.F;

  // RGB to/from YUV conversion coefficients
  Common::Vec3f YUVtoR({1.164F, 0.F, 1.596F});
  Common::Vec3f YUVtoG({1.164F, -0.392F, -0.813F});
  Common::Vec3f YUVtoB({1.164F, 2.017F, 0.F});
  Common::Vec3f RGBtoY({0.257F, 0.504F, 0.098F});
  Common::Vec3f RGBtoU({-0.148F, -0.291F, 0.439F});
  Common::Vec3f RGBtoV({0.439F, -0.368F, -0.071F});
  Common::Vec3f Cte({64.F, 512.F, 512.F});

  // YUV clamping :
  //   over  8 bits : Y is in range [16, 235], UV is in range [16, 240]
  //   over 10 bits : Y is in range [64, 940], UV is in range [64, 960]
  // https://stackoverflow.com/questions/25804565/accurate-yuv-10-bits-to-8-bits-conversion

  // 1) get RGB from YUV, 2) then greyish it, 3) then back to YUV
  for (int32_t h = 0; h < height_Y; h++) {
    for (int32_t w = 0; w < width_Y; w++) {
      const auto Y_h_w = static_cast<float>(Y(h, w));
      const auto U_h_w = static_cast<float>(U(h, w));
      const auto V_h_w = static_cast<float>(V(h, w));

      R = std::min(std::max((Y_h_w - Cte[0]) * YUVtoR[0] + (U_h_w - Cte[1]) * YUVtoR[1] +
                                (V_h_w - Cte[2]) * YUVtoR[2],
                            0.F),
                   maxValueF) *
          weight;
      G = std::min(std::max((Y_h_w - Cte[0]) * YUVtoG[0] + (U_h_w - Cte[1]) * YUVtoG[1] +
                                (V_h_w - Cte[2]) * YUVtoG[2],
                            0.F),
                   maxValueF) *
          weight;
      B = std::min(std::max((Y_h_w - Cte[0]) * YUVtoB[0] + (U_h_w - Cte[1]) * YUVtoB[1] +
                                (V_h_w - Cte[2]) * YUVtoB[2],
                            0.F),
                   maxValueF) *
          weight;

      Y(h, w) = Common::assertDownCast<uint16_t>(
          std::min(std::max(R * RGBtoY[0] + G * RGBtoY[1] + B * RGBtoY[2] + Cte[0], 64.F), 940.F));
      U(h, w) = Common::assertDownCast<uint16_t>(
          std::min(std::max(R * RGBtoU[0] + G * RGBtoU[1] + B * RGBtoU[2] + Cte[1], 64.F), 960.F));
      V(h, w) = Common::assertDownCast<uint16_t>(
          std::min(std::max(R * RGBtoV[0] + G * RGBtoV[1] + B * RGBtoV[2] + Cte[2], 64.F), 960.F));
    }
  }
}

} // namespace

ViewingSpaceController::ViewingSpaceController(const Common::Json & /*rootNode*/,
                                               const Common::Json & /*componentNode*/) {}

void ViewingSpaceController::inplaceFading(Common::RendererFrame &viewport,
                                           const MivBitstream::ViewParams &viewportParams,
                                           const MivBitstream::ViewingSpace &viewingSpace) const {
  inplaceFading_impl(viewport, viewportParams, computeIndex(viewportParams, viewingSpace));
}

} // namespace TMIV::Renderer
