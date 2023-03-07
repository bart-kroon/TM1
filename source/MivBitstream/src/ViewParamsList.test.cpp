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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <TMIV/MivBitstream/ViewParamsList.h>

using namespace std::string_literals;

TEST_CASE("Pose") {
  SECTION("En- and decoding between pose and camera extrinsics") {
    using TMIV::MivBitstream::Pose;

    auto unit = Pose{};
    unit.position = TMIV::Common::Vec3f{1.F, 2.F, 3.F};
    unit.orientation = TMIV::Common::QuatD{0.3, 0.3, 0.1, 0.9};

    const auto expectedCameraExtrinsics = []() {
      auto ce = TMIV::MivBitstream::CameraExtrinsics{};
      ce.ce_view_pos_x(1.F)
          .ce_view_pos_y(2.F)
          .ce_view_pos_z(3.F)
          .ce_view_quat_x(322122547)  // 0.299999999813735...
          .ce_view_quat_y(322122547)  // 0.299999999813735...
          .ce_view_quat_z(107374182); // 0.099999999627471...
      return ce;
    }();

    SECTION("Encode") {
      const auto actualCameraExtrinsics = unit.encodeToCameraExtrinsics();
      REQUIRE(actualCameraExtrinsics == expectedCameraExtrinsics);
    }

    SECTION("Decode") {
      const auto actualPose = Pose::decodeFrom(expectedCameraExtrinsics);

      REQUIRE(actualPose.position == unit.position);

      const auto approx = [](const auto x) {
        return Catch::Approx(x).epsilon(0).margin(std::ldexp(1, -31));
      };
      REQUIRE(actualPose.orientation.x() == approx(unit.orientation.x()));
      REQUIRE(actualPose.orientation.y() == approx(unit.orientation.y()));
      REQUIRE(actualPose.orientation.z() == approx(unit.orientation.z()));
      REQUIRE(actualPose.orientation.w() == approx(unit.orientation.w()));
    }

    SECTION("Print") {
      std::ostringstream stream;
      unit.printTo(stream, 1);

      const auto actualText = stream.str();
      const auto *referenceText = R"(position[ 1 ]=(1, 2, 3)
orientation[ 1 ]=0.9 + i 0.3 + j 0.3 + k 0.1
)";
      REQUIRE(actualText == referenceText);
    }
  }
}

TEST_CASE("ViewParams") {
  SECTION("Default construction with default values") {
    const auto unit = TMIV::MivBitstream::ViewParams{};

    CHECK(unit.pose == TMIV::MivBitstream::Pose{});
    CHECK(unit.ci == TMIV::MivBitstream::CameraIntrinsics{});
    CHECK(unit.dq == TMIV::MivBitstream::DepthQuantization{});
    CHECK(!unit.pp.has_value());
    CHECK(unit.name.empty());
    CHECK(!unit.hasOccupancy);
    CHECK(unit.isBasicView);
    CHECK(unit.nbMpiLayers == 1);
  }

  SECTION("Load from JSON, Equirectangular") {
    const auto json = TMIV::Common::Json::parse(R"(
{
    "Depth_range": [ 0.1, 500.0 ],
    "Hor_range": [ -90.0, 90.0 ],
    "Name": "v2",
    "Position": [ -0.2878679633140564, -0.0878679633140564, 1.0 ],
    "Projection": "Equirectangular",
    "Resolution": [ 2048, 1048 ],
    "Rotation": [ 45.00000125223908, 19.3, 4.3 ],
    "Ver_range": [ -90.0, 90.0 ]
})");

    auto unit = TMIV::MivBitstream::ViewParams(json);

    CHECK(unit.ci.ci_cam_type() == TMIV::MivBitstream::CiCamType::equirectangular);
    CHECK(unit.ci.ci_erp_phi_min() == -90.F);
    CHECK(unit.ci.ci_erp_phi_max() == 90.F);
    CHECK(unit.ci.ci_erp_theta_min() == -90.F);
    CHECK(unit.ci.ci_erp_theta_max() == 90.F);
    CHECK(unit.ci.ci_projection_plane_width_minus1() + 1 == 2048);
    CHECK(unit.ci.ci_projection_plane_height_minus1() + 1 == 1048);

    CHECK(unit.pose.position.x() == Catch::Approx(-0.2878679633140564));
    CHECK(unit.pose.position.y() == Catch::Approx(-0.0878679633140564));
    CHECK(unit.pose.position.z() == 1.);

    const auto q = TMIV::Common::euler2quat(
        TMIV::Common::Vec3d{TMIV::Common::deg2rad(45.00000125223908), TMIV::Common::deg2rad(19.3),
                            TMIV::Common::deg2rad(4.3)});

    CHECK(unit.pose.orientation.x() == Catch::Approx(q.x()));
    CHECK(unit.pose.orientation.y() == Catch::Approx(q.y()));
    CHECK(unit.pose.orientation.z() == Catch::Approx(q.z()));
    CHECK(unit.pose.orientation.w() == Catch::Approx(q.w()));

    CHECK(unit.dq.dq_norm_disp_low() == 2e-3F);
    CHECK(unit.dq.dq_norm_disp_high() == 10.F);

    CHECK(!unit.hasOccupancy);

    CHECK(unit.name == "v2");
  }

  SECTION("Load from JSON, Perspective") {
    const auto json = TMIV::Common::Json::parse(R"(
{
    "HasInvalidDepth": true,
    "Depth_range": [ 0.3, 1.62 ],
    "Focal": [ 1346.74, 1547.76 ],
    "Name": "v3",
    "Position": [ 0.000354626, 0.145079, -0.00036823 ],
    "Principle_point": [ 980.168, 534.722 ],
    "Projection": "Perspective",
    "Resolution": [ 1920, 1080 ],
    "Rotation": [ 0.0644106, -0.0149137, 5.85061e-05 ]
})");

    const auto unit = TMIV::MivBitstream::ViewParams{json};

    CHECK(unit.ci.ci_cam_type() == TMIV::MivBitstream::CiCamType::perspective);
    CHECK(unit.ci.ci_perspective_center_hor() == Catch::Approx(980.168));
    CHECK(unit.ci.ci_perspective_center_ver() == Catch::Approx(534.722));
    CHECK(unit.ci.ci_perspective_focal_hor() == Catch::Approx(1346.74));
    CHECK(unit.ci.ci_perspective_focal_ver() == Catch::Approx(1547.76));

    CHECK(unit.hasOccupancy);
  }

  SECTION("Load from JSON, MPI camera") {
    const auto json = TMIV::Common::Json::parse(R"(
{
    "Depth_range": [ 0.4160835445, 12.86596775 ],
    "Focal": [ 1749.296875, 1749.296875 ],
    "HasInvalidDepth": false,
    "Name": "mpi",
    "nbMpiLayers": 423,
    "Position": [ -2.499999762, 1.649999857, 1.449999332 ],
    "Principle_point": [ 2088.0, 1012.0 ],
    "Projection": "Perspective",
    "Resolution": [ 4176, 2024 ],
    "Rotation": [ -33.91259003, 7.734590054, 0.0 ]
})");

    const auto unit = TMIV::MivBitstream::ViewParams(json);

    CHECK(unit.ci.ci_cam_type() == TMIV::MivBitstream::CiCamType::perspective);
    CHECK(unit.ci.ci_perspective_center_hor() == Catch::Approx(2088.0));
    CHECK(unit.ci.ci_perspective_center_ver() == Catch::Approx(1012.0));
    CHECK(unit.nbMpiLayers == 423);
  }

  SECTION("Load from JSON, open depth range") {
    const auto json = TMIV::Common::Json::parse(R"(
{
    "Depth_range": [ 1E-4, "inf" ],
    "Hor_range": [ -90.0, 90.0 ],
    "Name": "v2",
    "Position": [ 0, 0, 0 ],
    "Projection": "Equirectangular",
    "Resolution": [ 2048, 1048 ],
    "Rotation": [ 0, 0, 0 ],
    "Ver_range": [ -90.0, 90.0 ]
})");

    const auto unit = TMIV::MivBitstream::ViewParams{json};

    REQUIRE(unit.dq.dq_norm_disp_low() == 0.F);
    REQUIRE(unit.dq.dq_norm_disp_high() == 1E4F);
  }

  SECTION("Load from JSON, open and inverted depth range") {
    const auto json = TMIV::Common::Json::parse(R"(
{
    "Depth_range": [ "inf", 0.1 ],
    "Hor_range": [ -90.0, 90.0 ],
    "Name": "v2",
    "Position": [ 0, 0, 0 ],
    "Projection": "Equirectangular",
    "Resolution": [ 2048, 1048 ],
    "Rotation": [ 0, 0, 0 ],
    "Ver_range": [ -90.0, 90.0 ]
})");

    const auto unit = TMIV::MivBitstream::ViewParams{json};

    REQUIRE(unit.dq.dq_norm_disp_low() == 10.F);
    REQUIRE(unit.dq.dq_norm_disp_high() == 0.F);
  }
}
