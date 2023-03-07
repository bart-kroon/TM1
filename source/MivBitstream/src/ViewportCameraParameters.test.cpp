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

#include "test.h"

#include <TMIV/MivBitstream/ViewportCameraParameters.h>

namespace TMIV::MivBitstream {
namespace examples {
const auto viewportCameraParameters =
    std::array{ViewportCameraParameters{},
               ViewportCameraParameters{1, false},
               ViewportCameraParameters{1, true},
               ViewportCameraParameters{1, false, true},
               ViewportCameraParameters{1, false, false, CiCamType::equirectangular},
               ViewportCameraParameters{1, false, false, CiCamType::orthographic},
               ViewportCameraParameters{1, false, false, CiCamType::perspective}};
} // namespace examples

TEST_CASE("View camera parameters coding") {
  REQUIRE(bitCodingTest(examples::viewportCameraParameters[0], 11));
  REQUIRE(bitCodingTest(examples::viewportCameraParameters[1], 143));
  REQUIRE(bitCodingTest(examples::viewportCameraParameters[2], 11));
  REQUIRE(bitCodingTest(examples::viewportCameraParameters[3], 143));
  REQUIRE(bitCodingTest(examples::viewportCameraParameters[4], 143));
  REQUIRE(bitCodingTest(examples::viewportCameraParameters[5], 143));
  REQUIRE(bitCodingTest(examples::viewportCameraParameters[6], 143));
}

TEST_CASE("View camera parameters perspective") {
  ViewportCameraParameters vcp{};
  float w = 1920.F;
  float h = 1080.F;
  float fov = 60.F;
  vcp.vcp_camera_id = 7;
  vcp.vcp_cancel_flag = false;
  vcp.vcp_persistent_flag = false;
  vcp.vcp_camera_type = CiCamType::perspective;
  vcp.vcp_perspective_aspect_ratio = w / h;
  vcp.vcp_perspective_horizontal_fov =
      std::clamp(static_cast<uint32_t>(std::round(fov * 65536.F)), 0U, 180U * 65536U - 1U);

  REQUIRE(bitCodingTest(vcp, 143));
}

TEST_CASE("View camera parameters from view params") {
  const auto json = TMIV::Common::Json::parse(R"(
{
    "Depth_range": [ 0.1, 500.0 ],
    "Hor_range": [ -90.0, 90.0  ],
    "Name": "v2",
    "Position": [ -0.2878679633140564, -0.0878679633140564, 1.0 ],
    "Projection": "Equirectangular",
    "Resolution": [ 2048, 1048 ],
    "Rotation": [ 45.00000125223908, 19.3, 4.3 ],
    "Ver_range": [ -90.0, 90.0 ]
})");

  auto vp = TMIV::MivBitstream::ViewParams(json);
  ViewportCameraParameters unit = ViewportCameraParameters::fromViewParams(vp);

  REQUIRE(bitCodingTest(unit, 143));
}
} // namespace TMIV::MivBitstream
