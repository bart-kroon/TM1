/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <TMIV/Renderer/Engine.h>

using Catch::Matchers::WithinAbs;
using TMIV::Common::Vec2f;
using TMIV::Common::Vec3f;
using TMIV::MivBitstream::CameraIntrinsics;
using TMIV::MivBitstream::CiCamType;
using TMIV::Renderer::SceneVertexDescriptor;

TEST_CASE("Engine<equirectangular>") {
  const auto unit = []() {
    auto ci = CameraIntrinsics{};
    ci.ci_cam_type(CiCamType::equirectangular)
        .ci_projection_plane_width_minus1(999)
        .ci_projection_plane_height_minus1(499)
        .ci_erp_phi_min(-90.F)   //   phi = -pi/2 --> u = 1000
        .ci_erp_phi_max(180.F)   //   phi =  pi   --> u =    0
        .ci_erp_theta_min(0.F)   // theta =  0    --> v =  500
        .ci_erp_theta_max(90.F); // theta =  pi/2 --> v =    0
    return TMIV::Renderer::Engine<TMIV::MivBitstream::CiCamType::equirectangular>{ci};
  }();

  SECTION("Reprojection accuracy test") {
    const auto depth = GENERATE(0.1F, 10.F);
    int32_t count = 0;

    for (int32_t i = 0; i < 500; ++i) {
      for (int32_t j = 0; j < 1000; ++j) {
        if (i * j % 1547 == 0) {
          const auto position =
              Vec2f{0.5F + static_cast<float>(j), 0.5F + static_cast<float>(i) + 0.5F};
          const auto p = unit.unprojectVertex(position, depth);
          const auto q = unit.projectVertex(SceneVertexDescriptor{p, 0.F});

          REQUIRE_THAT(q.depth, WithinAbs(depth, 1E-6));
          REQUIRE_THAT(q.position.x(), WithinAbs(position.x(), 2E-4));
          REQUIRE_THAT(q.position.y(), WithinAbs(position.y(), 1E-4));

          // NOTE(BK, #148): The margin on y is 1E-4 with atan2 and 5E-3 with asin (50x worse)
          // This can be easily understood by studying the derivatives, but it was measured using
          // this test section, and now this accuracy level is locked in.

          ++count;
        }
      }
    }

    REQUIRE(count == 3103);
  }
}
