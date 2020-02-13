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

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::DepthOccupancy;

SCENARIO("Depth/occupancy coding") {
  DepthOccupancy depthOccupancy{37};

  GIVEN("View parameters without invalid depth") {
    const auto projection = ErpParams{{-180.F, 180.F}, {-90.F, 90.F}};
    const auto sourceViewParams = ViewParams{{1920, 1080}, {}, {}, projection, {0.2F, 2.2F}, 0};
    const auto sourceSequenceParams = IvSequenceParams{{}, ViewParamsList{{sourceViewParams}}};

    WHEN("Modifying the depth range") {
      const auto codedSequenceParams = depthOccupancy.transformSequenceParams(sourceSequenceParams);

      THEN("The camera parameters are unmodified") {
        REQUIRE(codedSequenceParams == sourceSequenceParams);
      }
    }
  }

  GIVEN("View parameters with invalid depth") {
    const auto projection = ErpParams{{-180.F, 180.F}, {-90.F, 90.F}};
    auto sourceViewParams = ViewParams{{1920, 1080}, {}, {}, projection, {0.2F, 2.2F}, 0};
    sourceViewParams.hasOccupancy = true;
    const auto sourceSeqParams = IvSequenceParams{{}, ViewParamsList{{sourceViewParams}}};

    WHEN("Modifying the depth range") {
      const auto codedSeqParams = depthOccupancy.transformSequenceParams(sourceSeqParams);
      const auto &codedViewParams = codedSeqParams.viewParamsList.front();

      THEN("depthOccMapThreshold (T) >> 0") {
        const auto T = codedViewParams.depthOccMapThreshold;
        REQUIRE(T >= 8);

        THEN("Coded level 2T matches with source level 0") {
          // Output level 2T .. 1023 --> [0.2, 2.2] => rate = 2/(1023 - 2T), move 2T levels down
          const auto twoT = float(2 * T);
          const auto refViewParams = ViewParams{
              {1920, 1080}, {}, {}, projection, {0.2F - twoT * 2.F / (1023.F - twoT), 2.2F}, T};
          REQUIRE(codedSeqParams.viewParamsList.front() == refViewParams);
        }
      }
    }
  }
}
