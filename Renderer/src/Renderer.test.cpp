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

#include <TMIV/Renderer/AccumulatingPixel.h>
#include <TMIV/Renderer/Engine.h>
#include <TMIV/Renderer/Inpainter.h>
#include <TMIV/Renderer/MultipassRenderer.h>
#include <TMIV/Renderer/Rasterizer.h>
#include <TMIV/Renderer/Synthesizer.h>
#include <TMIV/Renderer/reprojectPoints.h>

#include <algorithm>
#include <cmath>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::Renderer;

auto makeFullERPCamera() {
  return ViewParams{{10, 5},                   // size
                    {1.F, 0.F, -1.F},          // position
                    {1.F, 2.F, -0.5F},         // orientation
                    ErpParams{{-180.F, 180.F}, // phi range
                              {-90.F, 90.F}},  // theta range
                    {1.F, 10.F}};              // depth range
}

TEST_CASE("Changing the reference frame", "[Render engine]") {
  const ViewParams neutral{};
  const ViewParams translated{{}, {1.F, 2.F, 3.F}};
  const ViewParams rotated{{}, {}, {100.F, 30.F, -30.F}};
  SECTION("trivial") {
    auto R_t = affineParameters(neutral, neutral);
    REQUIRE(R_t.first == Mat3x3f::eye());
    REQUIRE(R_t.second == Vec3f::zeros());
  }
  SECTION("translation") {
    auto R_t = affineParameters(neutral, translated);
    REQUIRE(R_t.first == Mat3x3f::eye());
    REQUIRE(R_t.second == -translated.position);
  }
  SECTION("rotation") {
    auto R_t = affineParameters(neutral, rotated);
    REQUIRE(none_of(begin(R_t.first), end(R_t.first), [](auto x) { return x == 0.F; }));
    REQUIRE(R_t.second == Vec3f::zeros());
  }
}

SCENARIO("Pixel can be blended", "[AccumulatingPixel]") {
  using Pixel = AccumulatingPixel<Vec3f>;
  using Acc = PixelAccumulator<Vec3f>;
  using Value = PixelValue<Vec3f>;

  GIVEN("A default-constructed accumulator") {
    Acc acc;
    Pixel pixel{1.F, 1.F, 1.F, 10.F};

    THEN("The attributes are zero")
    REQUIRE(get<0>(acc.attributes()).x() == 0.F);
    REQUIRE(get<0>(acc.attributes()).y() == 0.F);
    REQUIRE(get<0>(acc.attributes()).z() == 0.F);

    WHEN("Averaging") {
      auto val = pixel.average(acc);

      THEN("The attributes are zero") {
        REQUIRE(get<0>(val.attributes()).x() == 0.F);
        REQUIRE(get<0>(val.attributes()).y() == 0.F);
        REQUIRE(get<0>(val.attributes()).z() == 0.F);
      }
    }
  }

  GIVEN("A pixel accumulator that is constructed from a pixel value") {
    float const ray_angle_param = 1.5F;
    float const depth_param = 60.7F;
    float const stretching_param = 3.2F;
    float const max_stretching = 10.F;
    float const ray_angle = 0.01F;
    float const stretching = 3.F;
    Pixel pixel{ray_angle_param, depth_param, stretching_param, max_stretching};

    float const ray_angle_weight = pixel.rayAngleWeight(ray_angle);
    float const stretching_weight = pixel.stretchingWeight(stretching);

    Value reference{{{0.3F, 0.7F, 0.1F}}, 0.53F, ray_angle_weight * stretching_weight, stretching};

    Acc accum = pixel.construct(reference.attributes(), reference.normDisp, ray_angle, stretching);

    WHEN("The pixel value is directly computed") {
      Value actual = pixel.average(accum);

      THEN("The average is the pixel value") {
        REQUIRE(get<0>(actual.attributes())[0] == get<0>(reference.attributes())[0]);
        REQUIRE(get<0>(actual.attributes())[1] == get<0>(reference.attributes())[1]);
        REQUIRE(get<0>(actual.attributes())[2] == get<0>(reference.attributes())[2]);
        REQUIRE(actual.normDisp == reference.normDisp);
        REQUIRE(actual.normWeight == reference.normWeight);
      }
    }

    WHEN("The pixel is blended with itself") {
      Acc accum2 = pixel.blend(accum, accum);
      Value actual = pixel.average(accum2);

      THEN("The average is the same but with double quality") {
        REQUIRE(get<0>(actual.attributes())[0] == get<0>(reference.attributes())[0]);
        REQUIRE(get<0>(actual.attributes())[1] == get<0>(reference.attributes())[1]);
        REQUIRE(get<0>(actual.attributes())[2] == get<0>(reference.attributes())[2]);
        REQUIRE(actual.normDisp == reference.normDisp);
        REQUIRE(actual.normWeight == 2 * reference.normWeight);
      }
    }

    WHEN("The pixel is blended with another pixel that has invalid depth") {
      Acc accumInvalid = pixel.construct(reference.attributes(), 0.F, ray_angle, stretching);
      Value actual = pixel.average(pixel.blend(accum, accumInvalid));

      THEN("It is af the pixel has not been blended") {
        REQUIRE(get<0>(actual.attributes())[0] == get<0>(reference.attributes())[0]);
        REQUIRE(get<0>(actual.attributes())[1] == get<0>(reference.attributes())[1]);
        REQUIRE(get<0>(actual.attributes())[2] == get<0>(reference.attributes())[2]);
        REQUIRE(actual.normDisp == reference.normDisp);
        REQUIRE(actual.normWeight == reference.normWeight);
      }
    }
  }
}

SCENARIO("Reprojecting points", "[reprojectPoints]") {
  GIVEN("A camera and a depth map") {
    auto viewParams = makeFullERPCamera();
    Mat<float> depth({5U, 10U});
    fill(begin(depth), end(depth), 2.F);

    WHEN("Calculating image positions") {
      auto positions = imagePositions(viewParams);

      THEN("The image positions should be at the pixel centers") {
        REQUIRE(positions(4, 7).x() == 7.5F);
        REQUIRE(positions(0, 0).x() == 0.5F);
        REQUIRE(positions(0, 0).y() == 0.5F);
        REQUIRE(positions(4, 9).x() == 9.5F);
        REQUIRE(positions(4, 9).y() == 4.5F);
      }

      WHEN("Reprojecting points to the same camera with valid depth "
           "values") {
        auto actual = reprojectPoints(viewParams, viewParams, positions, depth);

        THEN("The positions should not change too much") {
          REQUIRE(actual.first(4, 7).x() == Approx(7.5F));
          REQUIRE(actual.first(0, 0).x() == Approx(0.5F));
          REQUIRE(actual.first(0, 0).y() == Approx(0.5F).margin(1e-4));
          REQUIRE(actual.first(4, 9).x() == Approx(9.5F));
          REQUIRE(actual.first(4, 9).y() == Approx(4.5F));
        }

        THEN("The depth values should not change too much") {
          REQUIRE(actual.second(4, 7) == Approx(2.F));
          REQUIRE(actual.second(0, 0) == Approx(2.F));
          REQUIRE(actual.second(0, 0) == Approx(2.F));
          REQUIRE(actual.second(4, 9) == Approx(2.F));
          REQUIRE(actual.second(4, 9) == Approx(2.F));
        }
      }
    }
  }
}

SCENARIO("Rastering meshes with 16-bit color as attribute", "[Rasterizer]") {
  GIVEN("A new rasterizer") {
    AccumulatingPixel<Vec3w> pixel{1.F, 1.F, 1.F, 10.F};
    Rasterizer<Vec3w> rasterizer(pixel, Vec2i{8, 4});

    WHEN("Rastering nothing") {
      THEN("The depth map is a matrix of NaN's or Inf's") {
        auto depth = rasterizer.depth();
        static_assert(is_same_v<decltype(depth), Mat<float>>);
        REQUIRE(depth.sizes() == array{size_t(4), size_t(8)});
        REQUIRE(none_of(begin(depth), end(depth), [](float x) { return isfinite(x); }));
      }
      THEN("The normalized disparity map is a matrix of zeroes") {
        auto normDisp = rasterizer.normDisp();
        static_assert(is_same_v<decltype(normDisp), Mat<float>>);
        REQUIRE(normDisp.sizes() == array{size_t(4), size_t(8)});
        REQUIRE(all_of(begin(normDisp), end(normDisp), [](float x) { return x == 0.F; }));
      }
      THEN("The normalized weight (quality) map is a matrix of zeros") {
        auto normWeight = rasterizer.normWeight();
        static_assert(is_same_v<decltype(normWeight), Mat<float>>);
        REQUIRE(normWeight.sizes() == array{size_t(4), size_t(8)});
        REQUIRE(all_of(begin(normWeight), end(normWeight), [](float x) { return x == 0.F; }));
      }
      THEN("The color map is a matrix of zero vectors") {
        auto color = rasterizer.attribute<0>();
        static_assert(is_same_v<decltype(color), Mat<Vec3w>>);
        REQUIRE(color.sizes() == array{size_t(4), size_t(8)});
        REQUIRE(all_of(begin(color), end(color), [](Vec3w x) { return x == Vec3w{}; }));
      }
    }
    WHEN("Submitting meshes but not rastering") {
      ImageVertexDescriptorList vs;
      TriangleDescriptorList ts;
      vector<Vec3w> as;
      rasterizer.submit(vs, tuple{as}, ts);
      THEN("Requesting output maps throws a Rasterizer::Exception") {
        using Ex = Rasterizer<Vec3w>::Exception;
        REQUIRE_THROWS_AS(rasterizer.depth(), Ex);
        REQUIRE_THROWS_AS(rasterizer.normDisp(), Ex);
        REQUIRE_THROWS_AS(rasterizer.normWeight(), Ex);
        REQUIRE_THROWS_AS(rasterizer.attribute<0>(), Ex);
      }
    }
    WHEN("Rastering some empty meshes") {
      ImageVertexDescriptorList vs;
      TriangleDescriptorList ts;
      vector<Vec3w> as;
      rasterizer.submit(vs, tuple{as}, ts);
      rasterizer.submit(vs, tuple{as}, ts);
      rasterizer.run();

      THEN("The depth map is a matrix of NaN's or Inf's") {
        auto depth = rasterizer.depth();
        static_assert(is_same_v<decltype(depth), Mat<float>>);
        REQUIRE(depth.sizes() == array{size_t(4), size_t(8)});
        REQUIRE(none_of(begin(depth), end(depth), [](float x) { return isfinite(x); }));
      }
      THEN("The normalized disparity map is a matrix of zeroes") {
        auto normDisp = rasterizer.normDisp();
        static_assert(is_same_v<decltype(normDisp), Mat<float>>);
        REQUIRE(normDisp.sizes() == array{size_t(4), size_t(8)});
        REQUIRE(all_of(begin(normDisp), end(normDisp), [](float x) { return x == 0.F; }));
      }
      THEN("The normalized weight (quality) map is a matrix of zeros") {
        auto normWeight = rasterizer.normWeight();
        static_assert(is_same_v<decltype(normWeight), Mat<float>>);
        REQUIRE(normWeight.sizes() == array{size_t(4), size_t(8)});
        REQUIRE(all_of(begin(normWeight), end(normWeight), [](float x) { return x == 0.F; }));
      }
      THEN("The color map is a matrix of zero vectors") {
        auto color = rasterizer.attribute<0>();
        static_assert(is_same_v<decltype(color), Mat<Vec3w>>);
        REQUIRE(color.sizes() == array{size_t(4), size_t(8)});
        REQUIRE(all_of(begin(color), end(color), [](Vec3w x) { return x == Vec3w{}; }));
      }
    }
    WHEN("Rastering a quad") {
      // Rectangle of 2 x 6 pixels within the 4 x 8 pixels frame
      // Depth values misused to correspond to x-values
      // Ray angles misused to correspond to y-values
      ImageVertexDescriptorList vs{
          {{1, 1}, 1.F, 1.F}, {{7, 1}, 7.F, 1.F}, {{7, 3}, 7.F, 3.F}, {{1, 3}, 1.F, 3.F}};
      // Two clockwise triangles
      TriangleDescriptorList ts{{{0, 1, 2}, 3.F}, {{0, 2, 3}, 5.F}};

      // Colors correspond to x and y-values times 100
      vector<Vec3w> as{{100, 0, 100}, {700, 0, 100}, {700, 0, 300}, {100, 0, 300}};
      rasterizer.submit(vs, tuple{as}, ts);
      rasterizer.run();

      THEN("The depth map has known values") {
        auto depth = rasterizer.depth();
        REQUIRE(!isfinite(depth(0, 0)));
        REQUIRE(depth(1, 1) == 1.F / (11.F / 12.F + 1.F / 12.F / 7.F));
        REQUIRE(depth(1, 5) == 1.F / (3.F / 12.F + 9.F / 12.F / 7.F));
        REQUIRE(!isfinite(depth(2, 7)));
        REQUIRE(!isfinite(depth(3, 7)));
      }
      THEN("The normalized disparity map has known values") {
        auto normDisp = rasterizer.normDisp();
        REQUIRE(normDisp(0, 0) == 0.F);
        REQUIRE(normDisp(1, 1) == 11.F / 12.F + 1.F / 12.F / 7.F);
        REQUIRE(normDisp(1, 5) == 3.F / 12.F + 9.F / 12.F / 7.F);
        REQUIRE(normDisp(2, 7) == 0.F);
        REQUIRE(normDisp(3, 7) == 0.F);
      }
      THEN("The normalized weight (quality) has known values for except for "
           "points that intersect triangle edges") {
        auto normWeight = rasterizer.normWeight();

        // The average ray angle of all three vertices is used for all
        // points in a triangle Note that ray angles are artifical examples.
        // Typical values would be <0.1 rad.
        const float rayAngle1 = 1.F * (2.F / 3.F) + 3.F * (1.F / 3.F);
        const float rayAngle2 = 1.F * (1.F / 3.F) + 3.F * (2.F / 3.F);
        const float w_rayAngle1 = pixel.rayAngleWeight(rayAngle1);
        const float w_rayAngle2 = pixel.rayAngleWeight(rayAngle2);

        // The same stretching weight is used for all points in a triangle.
        // The stretching is the ratio of original and synthesized areas.
        const float w_stretching1 = pixel.stretchingWeight(6.F / 3.F);
        const float w_stretching2 = pixel.stretchingWeight(6.F / 5.F);

        // The weight is normalized by the depth weight, so effectively:
        const float w_normWeight1 = w_rayAngle1 * w_stretching1;
        const float w_normWeight2 = w_rayAngle2 * w_stretching2;

        REQUIRE(normWeight(0, 0) == 0.F);
        REQUIRE(normWeight(1, 1) == Approx(w_normWeight2));
        REQUIRE(normWeight(1, 5) == Approx(w_normWeight1));
        REQUIRE(normWeight(2, 7) == 0.F);
        REQUIRE(normWeight(3, 7) == 0.F);

        THEN("Points that intersect triangle edges are interpolated") {
          REQUIRE(normWeight(1, 2) == Approx((w_normWeight2 + w_normWeight1) / 2));
        }
      }
      THEN("The color map has known values") {
        const auto color = rasterizer.attribute<0>();
        REQUIRE(color(0, 0) == Vec3w{});
        REQUIRE(color(1, 1) == Vec3w{150, 0, 150});
        REQUIRE(color(1, 5) == Vec3w{550, 0, 150});
        REQUIRE(color(2, 7) == Vec3w{});
        REQUIRE(color(3, 7) == Vec3w{});

        WHEN("Submitting more meshes but not rastering") {
          rasterizer.submit({}, {}, {});
          THEN("Requesting output maps throws a Rasterizer::Exception") {
            using Ex = Rasterizer<Vec3w>::Exception;
            REQUIRE_THROWS_AS(rasterizer.depth(), Ex);
            REQUIRE_THROWS_AS(rasterizer.normDisp(), Ex);
            REQUIRE_THROWS_AS(rasterizer.normWeight(), Ex);
            REQUIRE_THROWS_AS(rasterizer.attribute<0>(), Ex);
          }
        }
        WHEN("Rastering more meshes") {
          rasterizer.submit({}, {}, {});
          rasterizer.run();

          THEN("This is cumulative") {
            auto color2 = rasterizer.attribute<0>();
            REQUIRE(color2(0, 0) == Vec3w{});
            REQUIRE(color2(1, 1) == Vec3w{150, 0, 150});
            REQUIRE(color2(1, 5) == Vec3w{550, 0, 150});
            REQUIRE(color2(2, 7) == Vec3w{});
            REQUIRE(color2(3, 7) == Vec3w{});
          }
        }
      }
    }
  }
}

SCENARIO("Rastering meshes with Vec2f as attribute", "[Rasterizer]") {
  GIVEN("A new rasterizer") {
    Rasterizer<Vec2f> rasterizer(AccumulatingPixel<Vec2f>{1.F, 1.F, 1.F, 10.F}, Vec2i{8, 4});

    WHEN("Rastering nothing") {
      THEN("The field map is a matrix of zero vectors") {
        auto field = rasterizer.attribute<0>();
        static_assert(is_same_v<decltype(field), Mat<Vec2f>>);
        REQUIRE(field.sizes() == array{size_t(4), size_t(8)});
        REQUIRE(all_of(begin(field), end(field), [](Vec2f x) { return x == Vec2f{}; }));
      }
    }
    WHEN("Rastering some empty meshes") {
      ImageVertexDescriptorList vs;
      TriangleDescriptorList ts;
      vector<Vec2f> as;
      rasterizer.submit(vs, tuple{as}, ts);
      rasterizer.submit(vs, tuple{as}, ts);
      rasterizer.run();

      THEN("The field map is a matrix of zero vectors") {
        auto field = rasterizer.attribute<0>();
        static_assert(is_same_v<decltype(field), Mat<Vec2f>>);
        REQUIRE(field.sizes() == array{size_t(4), size_t(8)});
        REQUIRE(all_of(begin(field), end(field), [](Vec2f x) { return x == Vec2f{}; }));
      }
    }
    WHEN("Rastering a quad") {
      // Rectangle of 2 x 6 pixels within the 4 x 8 pixels frame
      // Depth values misused to correspond to x-values
      // Ray angles misused to correspond to y-values
      ImageVertexDescriptorList vs{
          {{1, 1}, 1.F, 1.F}, {{7, 1}, 7.F, 1.F}, {{7, 3}, 7.F, 3.F}, {{1, 3}, 1.F, 3.F}};

      // Two clockwise triangles
      TriangleDescriptorList ts{{{0, 1, 2}, 6.F}, {{0, 2, 3}, 6.F}};

      // Colors correspond to x and y-values times 100
      vector<Vec2f> as{{100.F, 100.F}, {700.F, 100.F}, {700.F, 300.F}, {100.F, 300.F}};
      rasterizer.submit(vs, tuple{as}, ts);
      rasterizer.run();

      THEN("The field map has known values") {
        auto field = rasterizer.attribute<0>();
        REQUIRE(field(0, 0) == Vec2f{});
        REQUIRE(field(1, 1).x() == Approx(150.F));
        REQUIRE(field(1, 1).y() == Approx(150.F));
        REQUIRE(field(1, 5).x() == Approx(550.F));
        REQUIRE(field(1, 5).y() == Approx(150.F));
        REQUIRE(field(2, 7) == Vec2f{});
        REQUIRE(field(3, 7) == Vec2f{});

        WHEN("Rastering more meshes") {
          rasterizer.submit({}, {}, {});
          rasterizer.run();

          THEN("This is cumulative") {
            auto field2 = rasterizer.attribute<0>();
            REQUIRE(field2(0, 0) == Vec2f{});
            REQUIRE(field2(1, 1).x() == Approx(150.F));
            REQUIRE(field2(1, 1).y() == Approx(150.F));
            REQUIRE(field2(1, 5).x() == Approx(550.F));
            REQUIRE(field2(1, 5).y() == Approx(150.F));
            REQUIRE(field2(2, 7) == Vec2f{});
            REQUIRE(field2(3, 7) == Vec2f{});
          }
        }
      }
    }
  }
}
