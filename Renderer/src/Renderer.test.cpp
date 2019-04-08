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

#include <catch2/catch.hpp>

#include "AccumulatingPixel.h"
#include <TMIV/Renderer/Inpainter.h>
#include <TMIV/Renderer/MultipassRenderer.h>
#include <TMIV/Renderer/Synthesizer.h>
#include <TMIV/Renderer/reprojectPoints.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::Renderer;

const auto NaN = numeric_limits<float>::quiet_NaN();
const auto inf = numeric_limits<float>::infinity();

SCENARIO("Pixel can be blended", "[AccumulatingPixel]") {
  using PA = AccumulatingPixel::PixelAccumulator;
  using PV = AccumulatingPixel::PixelValue;

  GIVEN("A pixel accumulator that is constructed from a pixel value") {
    float const ray_angle_param = 1.5f;
    float const depth_param = 60.7f;
    float const stretching_param = 3.2f;
    float const ray_angle = 0.01f;
    float const stretching = 3.f;

    AccumulatingPixel pixel{ray_angle_param, depth_param, stretching_param,
                            AccumulatingPixel::Mode::all};

    float const ray_angle_weight = pixel.rayAngleWeight(ray_angle);
    float const stretching_weight = pixel.stretchingWeight(stretching);

    PV reference{{0.3f, 0.7f, 0.1f},
                 0.53f,
                 ray_angle_weight * stretching_weight,
                 stretching_weight};

    PA accum = pixel.construct(reference.depth, reference.color, ray_angle,
                               stretching);

    WHEN("The pixel value is directly computed") {
      PV actual = pixel.average(accum);

      THEN("The average is the pixel value") {
        REQUIRE(actual.color[0] == reference.color[0]);
        REQUIRE(actual.color[1] == reference.color[1]);
        REQUIRE(actual.color[2] == reference.color[2]);
        REQUIRE(actual.depth == reference.depth);
        REQUIRE(actual.quality == reference.quality);
        REQUIRE(actual.validity == reference.validity);
      }
    }

    WHEN("The pixel is blended with itself") {
      PA accum2 = pixel.blend(accum, accum);
      PV actual = pixel.average(accum2);

      THEN("The average is the same but with double quality") {
        REQUIRE(actual.color[0] == reference.color[0]);
        REQUIRE(actual.color[1] == reference.color[1]);
        REQUIRE(actual.color[2] == reference.color[2]);
        REQUIRE(actual.depth == reference.depth);
        REQUIRE(actual.quality == 2 * reference.quality);
        REQUIRE(actual.validity == reference.validity);
      }
    }

    WHEN("The pixel is blended with another pixel that has invalid depth") {
      const float NaN = numeric_limits<float>::quiet_NaN();
      PA accumNaN =
          pixel.construct(NaN, reference.color, ray_angle, stretching);
      PV actual = pixel.average(pixel.blend(accum, accumNaN));

      THEN("It is af the pixel has not been blended") {
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

SCENARIO("Reprojecting points", "[reprojectPoints]") {
  GIVEN("A camera and a depth map") {
    CameraParameters camera{{100, 50},         // size
                            {1.f, 0.f, -1.f},  // position
                            {1.f, 2.f, -0.5f}, // orientation
                            ProjectionType::ERP,
                            {-180.f, 180.f}, // phi range
                            {-90.f, 90.f},   // theta range
                            {},
                            {},
                            {},
                            {1.f, 10.f}}; // depth range

    Mat1f depth({50u, 100u});
    fill(begin(depth), end(depth), 2.f);

    WHEN("Calculating image positions") {
      auto positions = imagePositions(camera);

      THEN("The image positions should be at the pixel centers") {
        REQUIRE(positions(4, 7).x() == 7.5f);
        REQUIRE(positions(0, 0).x() == 0.5f);
        REQUIRE(positions(0, 0).y() == 0.5f);
        REQUIRE(positions(49, 99).x() == 99.5f);
        REQUIRE(positions(49, 99).y() == 49.5f);
      }

      WHEN("Reprojecting points to the same camera with valid depth values") {
        auto actual = reprojectPoints(camera, camera, positions, depth);

        THEN("The positions should not change too much") {
          REQUIRE(actual.first(4, 7).x() == Approx(7.5f));
          REQUIRE(actual.first(0, 0).x() == Approx(0.5f));
          REQUIRE(actual.first(0, 0).y() == Approx(0.5f).margin(1e-4));
          REQUIRE(actual.first(49, 99).x() == Approx(99.5f));
          REQUIRE(actual.first(49, 99).y() == Approx(49.5f));
        }

        THEN("The depth values should not change too much") {
          REQUIRE(actual.second(4, 7) == Approx(2.f));
          REQUIRE(actual.second(0, 0) == Approx(2.f));
          REQUIRE(actual.second(0, 0) == Approx(2.f));
          REQUIRE(actual.second(49, 99) == Approx(2.f));
          REQUIRE(actual.second(49, 99) == Approx(2.f));
        }
      }
    }
  }
}

SCENARIO("Synthesis of a depth map", "[Synthesizer]") {
  using Mat1f = TMIV::Common::Mat<float>;

  GIVEN("A synthesizer, camera and a depth map") {
    Synthesizer synthesizer{1., 1., 1.};

    CameraParameters camera{{100, 50},         // size
                            {1.f, 0.f, -1.f},  // position
                            {1.f, 2.f, -0.5f}, // orientation
                            ProjectionType::ERP,
                            {-180.f, 180.f}, // phi range
                            {-90.f, 90.f},   // theta range
                            {},
                            {},
                            {},
                            {1.f, 10.f}}; // depth range

    Mat1f depth({unsigned(camera.size.y()), unsigned(camera.size.x())});
    REQUIRE(depth.width() == 100);
    REQUIRE(depth.height() == 50);
    fill(begin(depth), end(depth), 2.f);

    WHEN("Synthesizing to the same viewpoint") {
      auto actual = synthesizer.renderDepth(depth, camera, camera);

      THEN("The output depth should match the input depth") {
        for (auto i = 0u; i != depth.height(); ++i) {
          for (auto j = 0u; j != depth.width(); ++j) {
            REQUIRE(actual(i, j) == Approx(2.f));
          }
        }
      }
    }
  }
}
