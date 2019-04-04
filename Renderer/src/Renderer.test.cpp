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

#include <Catch2/catch.hpp>

#include "AccumulatingPixel.h"
#include <TMIV/Renderer/Inpainter.h>
#include <TMIV/Renderer/MultipassRenderer.h>
#include <TMIV/Renderer/Synthesizer.h>

using namespace TMIV::Common;

SCENARIO("Pixel can be blended", "[AccumulatingPixel]") {
  using PA = TMIV::Renderer::AccumulatingPixel::PixelAccumulator;
  using PV = TMIV::Renderer::AccumulatingPixel::PixelValue;

  GIVEN("An accumulator with some parameters and a reference pixel value") {
    float const ray_angle_param = 1.5f;
    float const depth_param = 60.7f;
    float const stretching_param = 3.2f;
    float const ray_angle = 0.01f;
    float const stretching = 3.f;

    TMIV::Renderer::AccumulatingPixel pixel{ray_angle_param, depth_param,
                                            stretching_param};

    float const ray_angle_weight = pixel.rayAngleWeight(ray_angle);
    float const stretching_weight = pixel.stretchingWeight(stretching);

    PV reference{{0.3f, 0.7f, 0.1f},
                 0.53f,
                 ray_angle_weight * stretching_weight,
                 stretching_weight};

    WHEN("A pixel accumulator is constructed from a pixel value") {
      PA accum = pixel.construct(reference.depth, reference.color, ray_angle,
                                 stretching);
      THEN("The average is the pixel value") {
        PV actual = pixel.average(accum);
        REQUIRE(actual.color[0] == reference.color[0]);
        REQUIRE(actual.color[1] == reference.color[1]);
        REQUIRE(actual.color[2] == reference.color[2]);
        REQUIRE(actual.depth == reference.depth);
        REQUIRE(actual.quality == reference.quality);
        REQUIRE(actual.validity == reference.validity);
      }
    }
  }
}
