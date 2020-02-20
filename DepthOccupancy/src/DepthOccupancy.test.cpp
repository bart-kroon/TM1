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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <TMIV/DepthOccupancy/DepthOccupancy.h>

#include <TMIV/Common/Common.h>
#include <TMIV/MivBitstream/MivDecoder.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;
using namespace TMIV::DepthOccupancy;

namespace TMIV::MivBitstream {
const MivDecoder::Mode MivDecoder::mode = MivDecoder::Mode::MIV;
}

SCENARIO("Depth/occupancy coding") {
  DepthOccupancy depthOccupancy{37};

  auto sourceViewParams = ViewParams{};
  sourceViewParams.ci.ci_projection_plane_width_minus1(1919)
      .ci_projection_plane_height_minus1(1079)
      .ci_cam_type(CiCamType::equirectangular)
      .ci_erp_phi_min(-halfCycle)
      .ci_erp_phi_max(halfCycle)
      .ci_erp_theta_min(-quarterCycle)
      .ci_erp_theta_max(quarterCycle);
  sourceViewParams.dq.dq_norm_disp_low(0.2F).dq_norm_disp_high(2.2F);

  GIVEN("View parameters without invalid depth") {
    auto sourceSequenceParams = IvSequenceParams{};
    sourceSequenceParams.viewParamsList = ViewParamsList{{sourceViewParams}};

    WHEN("Modifying the depth range") {
      const auto codedSequenceParams = depthOccupancy.transformSequenceParams(sourceSequenceParams);

      THEN("The camera parameters are unmodified") {
        REQUIRE(codedSequenceParams == sourceSequenceParams);
      }
    }
  }

  GIVEN("View parameters with invalid depth") {
    sourceViewParams.hasOccupancy = true;
    auto sourceSeqParams = IvSequenceParams{};
    sourceSeqParams.viewParamsList = ViewParamsList{{sourceViewParams}};

    WHEN("Modifying the depth range") {
      const auto codedSeqParams = depthOccupancy.transformSequenceParams(sourceSeqParams);
      const auto &codedViewParams = codedSeqParams.viewParamsList.front();

      THEN("pduDepthOccMapThreshold (T) >> 0") {
        const auto T = codedViewParams.dq.dq_depth_occ_map_threshold_default();
        REQUIRE(T >= 8);

        THEN("Coded level 2T matches with source level 0") {
          // Output level 2T .. 1023 --> [0.2, 2.2] => rate = 2/(1023 - 2T), move 2T levels down
          const auto twoT = float(2 * T);

          auto refViewParams = sourceViewParams;
          refViewParams.dq.dq_depth_occ_map_threshold_default(T)
              .dq_norm_disp_low(0.2F - twoT * 2.F / (1023.F - twoT))
              .dq_norm_disp_high(2.2F);

          REQUIRE(codedSeqParams.viewParamsList.front() == refViewParams);
        }
      }
    }
  }
}
