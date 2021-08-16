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

#include <catch2/catch.hpp>

#include <TMIV/Encoder/GeometryQuantizer.h>

#include <TMIV/Common/Common.h>

SCENARIO("Explicit occupancy") {
  TMIV::Encoder::GeometryQuantizer explicitOccupancy{64};

  auto sourceParams = TMIV::Encoder::EncoderParams{};
  sourceParams.vps.vps_atlas_count_minus1(1)
      .vps_atlas_id(0, TMIV::MivBitstream::AtlasId(0))
      .vps_atlas_id(1, TMIV::MivBitstream::AtlasId(1))
      .vps_miv_extension()
      .vme_embedded_occupancy_enabled_flag(false);
  sourceParams.casps.casps_miv_extension().casme_depth_low_quality_flag(true);
  sourceParams.atlas.emplace_back();
  sourceParams.atlas[0].asps.asps_frame_width(1920);
  sourceParams.atlas[0].asps.asps_frame_height(4640);
  sourceParams.atlas[0].asps.asps_log2_patch_packing_block_size(5);
  sourceParams.atlas[0].asps.asps_miv_extension().asme_occupancy_scale_enabled_flag(true);
  sourceParams.atlas.emplace_back();
  sourceParams.atlas[1].asps.asps_frame_width(2048);
  sourceParams.atlas[1].asps.asps_frame_height(4352);
  sourceParams.atlas[1].asps.asps_log2_patch_packing_block_size(4);
  sourceParams.atlas[1].asps.asps_miv_extension().asme_occupancy_scale_enabled_flag(true);

  GIVEN("Signaling occupancy maps explicitly") {
    WHEN("Calling transformParams") {
      const auto codedParams = explicitOccupancy.transformParams(sourceParams);

      THEN("Encoder ASPS params are not modified") {
        REQUIRE(codedParams.atlas[0].asps == sourceParams.atlas[0].asps);
        REQUIRE(codedParams.atlas[1].asps == sourceParams.atlas[1].asps);
      }
    }

    WHEN("Calling setOccupancyParams") {
      const auto codedParams = explicitOccupancy.setOccupancyParams(sourceParams, true, true);

      THEN("Encoder ASPS params are modified to enable explicit occupancy") {
        REQUIRE(codedParams.vps.vps_miv_extension().vme_embedded_occupancy_enabled_flag() == false);
        REQUIRE(codedParams.vps.vps_miv_extension().vme_occupancy_scale_enabled_flag() == true);
        REQUIRE(codedParams.atlas[0].asps != sourceParams.atlas[0].asps);
        REQUIRE(
            codedParams.atlas[0].asps.asps_miv_extension().asme_occupancy_scale_enabled_flag() ==
            true);
        REQUIRE(
            codedParams.atlas[0].asps.asps_miv_extension().asme_occupancy_scale_factor_x_minus1() ==
            31);
        REQUIRE(
            codedParams.atlas[0].asps.asps_miv_extension().asme_occupancy_scale_factor_y_minus1() ==
            15);
        REQUIRE(codedParams.atlas[1].asps != sourceParams.atlas[1].asps);
        REQUIRE(
            codedParams.atlas[1].asps.asps_miv_extension().asme_occupancy_scale_enabled_flag() ==
            true);
        REQUIRE(
            codedParams.atlas[1].asps.asps_miv_extension().asme_occupancy_scale_factor_x_minus1() ==
            15);
        REQUIRE(
            codedParams.atlas[1].asps.asps_miv_extension().asme_occupancy_scale_factor_y_minus1() ==
            15);
      }
    }
  }
}
