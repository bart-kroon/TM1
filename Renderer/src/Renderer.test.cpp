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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include "AccumulatingPixel.h"
#include "Engine.h"
#include "Rasterizer.h"
#include <TMIV/Renderer/Inpainter.h>
#include <TMIV/Renderer/MultipassRenderer.h>
#include <TMIV/Renderer/Synthesizer.h>
#include <TMIV/Renderer/reprojectPoints.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::Renderer;

TEST_CASE("Full ERP", "[Render engine]") {
  Mat<float> depth({5, 7});
  fill(begin(depth), end(depth), 2.f);
  const CameraParameters camera{
      0,  {7, 5}, {}, {}, ProjectionType::ERP, {-180.f, 180.f}, {-90.f, 90.f},
      {}, {},     {}, {}};

  SECTION("Unproject without attributes") {
    auto mesh = unproject(depth, camera, camera);
    auto as = get<2>(mesh);
    static_assert(is_same_v<decltype(as), tuple<>>);
  }

  SECTION("Reproject without attributes") {
    auto mesh = reproject(depth, camera, camera);
    auto as = get<2>(mesh);
    static_assert(is_same_v<decltype(as), tuple<>>);
  }

  SECTION("Unproject with an attribute") {
    Mat<float> field({5, 7});
    fill(begin(field), end(field), 3.f);
    auto mesh = unproject(depth, camera, camera, field);

    auto vs = get<0>(mesh);
    REQUIRE(vs.size() == (7 + 1) * 5 + 2);
    for (auto v : vs) {
      REQUIRE(v.rayAngle == 0.f);
    }
    // Central vertex in forward (x) direction
    REQUIRE(vs[(5 / 2) * (7 + 1) + 7 / 2].position == Vec3f{2.f, 0.f, 0.f});

    auto ts = get<1>(mesh);
    REQUIRE(ts.size() == 2 * 7 * (5 - 1) + 2 * 7);

    auto as = get<2>(mesh);
    static_assert(is_same_v<decltype(as), tuple<vector<float>>>);
    REQUIRE(get<0>(as).size() == vs.size());
    REQUIRE(get<0>(as)[0] == 3.f);
  }

  SECTION("Reproject with an attribute") {
    Mat<float> field({5, 7});
    fill(begin(field), end(field), 3.f);
    auto mesh = reproject(depth, camera, camera, field);

    auto vs = get<0>(mesh);
    REQUIRE(vs.size() == (7 + 1) * 5 + 2);
    for (auto v : vs) {
      REQUIRE(v.depth == Approx(2.f));
      REQUIRE(v.rayAngle == 0.f);
    }
    REQUIRE(vs[0].position.x() == Approx(0.5f));
    REQUIRE(vs[0].position.y() == Approx(0.5f));

    auto ts = get<1>(mesh);
    REQUIRE(ts.size() == 2 * 7 * (5 - 1) + 2 * 7);

    auto as = get<2>(mesh);
    static_assert(is_same_v<decltype(as), tuple<vector<float>>>);
    REQUIRE(get<0>(as).size() == vs.size());
    REQUIRE(get<0>(as)[0] == 3.f);
  }
}

TEST_CASE("Equirectangular viewport", "[Render engine]") {
  Mat<float> depth({5, 7});
  fill(begin(depth), end(depth), 2.f);
  const CameraParameters camera{
      0,  {7, 5}, {}, {}, ProjectionType::ERP, {-10.f, 10.f}, {-10.f, 10.f},
      {}, {},     {}, {}};

  SECTION("Unproject without attributes") {
    auto mesh = unproject(depth, camera, camera);
    auto as = get<2>(mesh);
    static_assert(is_same_v<decltype(as), tuple<>>);
  }

  SECTION("Reproject without attributes") {
    auto mesh = reproject(depth, camera, camera);
    auto as = get<2>(mesh);
    static_assert(is_same_v<decltype(as), tuple<>>);
  }

  SECTION("Unproject with an attribute") {
    Mat<float> field({5, 7});
    fill(begin(field), end(field), 3.f);
    auto mesh = unproject(depth, camera, camera, field);

    auto vs = get<0>(mesh);
    REQUIRE(vs.size() == (7 + 2) * (5 + 2));
    for (auto v : vs) {
      REQUIRE(v.rayAngle == 0.f);
    }
    // Central vertex in forward (x) direction
    REQUIRE(vs[vs.size() / 2].position == Vec3f{2.f, 0.f, 0.f});

    auto ts = get<1>(mesh);
    REQUIRE(ts.size() == 2 * (7 + 1) * (5 + 1));

    auto as = get<2>(mesh);
    static_assert(is_same_v<decltype(as), tuple<vector<float>>>);
    REQUIRE(get<0>(as).size() == vs.size());
    REQUIRE(get<0>(as)[0] == 3.f);
  }

  SECTION("Reproject with an attribute") {
    Mat<float> field({5, 7});
    fill(begin(field), end(field), 3.f);
    auto mesh = reproject(depth, camera, camera, field);

    auto vs = get<0>(mesh);
    REQUIRE(vs.size() == (7 + 2) * (5 + 2));
    for (auto v : vs) {
      REQUIRE(v.depth == Approx(2.f));
      REQUIRE(v.rayAngle == 0.f);
    }
    REQUIRE(vs.front().position.x() == 0.f);
    REQUIRE(vs.back().position.y() == 5.f);

    auto ts = get<1>(mesh);
    REQUIRE(ts.size() == 2 * (7 + 1) * (5 + 1));

    auto as = get<2>(mesh);
    static_assert(is_same_v<decltype(as), tuple<vector<float>>>);
    REQUIRE(get<0>(as).size() == vs.size());
    REQUIRE(get<0>(as)[0] == 3.f);
  }
}

TEST_CASE("Perspective viewport", "[Render engine]") {
  Mat<float> depth({5, 7});
  fill(begin(depth), end(depth), 2.f);
  const CameraParameters camera{
      0,  {7, 5},       {},           {}, ProjectionType::Perspective, {}, {},
      {}, {10.f, 10.f}, {3.5f, 2.5f}, {}};
}

TEST_CASE("Changing the reference frame", "[Render engine]") {
  const CameraParameters neutral{};
  const CameraParameters translated{
      0, {}, {1.f, 2.f, 3.f}, {}, {}, {}, {}, {}, {}, {}, {}};
  const CameraParameters rotated{
      0, {}, {}, {100.f, 30.f, -30.f}, {}, {}, {}, {}, {}, {}, {}};
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
    REQUIRE(none_of(begin(R_t.first), end(R_t.first),
                    [](auto x) { return x == 0.f; }));
    REQUIRE(R_t.second == Vec3f::zeros());
  }
}

SCENARIO("Pixel can be blended", "[AccumulatingPixel]") {
  using Pixel = AccumulatingPixel<Vec3f>;
  using Acc = PixelAccumulator<Vec3f>;
  using Value = PixelValue<Vec3f>;

  GIVEN("A pixel accumulator that is constructed from a pixel value") {
    float const ray_angle_param = 1.5f;
    float const depth_param = 60.7f;
    float const stretching_param = 3.2f;
    float const ray_angle = 0.01f;
    float const stretching = 3.f;

    Pixel pixel{ray_angle_param, depth_param, stretching_param};

    float const ray_angle_weight = pixel.rayAngleWeight(ray_angle);
    float const stretching_weight = pixel.stretchingWeight(stretching);

    Value reference{
        0.53f, ray_angle_weight * stretching_weight, {0.3f, 0.7f, 0.1f}};

    Acc accum = pixel.construct(reference.attributes(), reference.normDisp,
                                ray_angle, stretching);

    WHEN("The pixel value is directly computed") {
      Value actual = pixel.average(accum);

      THEN("The average is the pixel value") {
        REQUIRE(get<0>(actual.attributes())[0] ==
                get<0>(reference.attributes())[0]);
        REQUIRE(get<0>(actual.attributes())[1] ==
                get<0>(reference.attributes())[1]);
        REQUIRE(get<0>(actual.attributes())[2] ==
                get<0>(reference.attributes())[2]);
        REQUIRE(actual.normDisp == reference.normDisp);
        REQUIRE(actual.normWeight == reference.normWeight);
      }
    }

    WHEN("The pixel is blended with itself") {
      Acc accum2 = pixel.blend(accum, accum);
      Value actual = pixel.average(accum2);

      THEN("The average is the same but with double quality") {
        REQUIRE(get<0>(actual.attributes())[0] ==
                get<0>(reference.attributes())[0]);
        REQUIRE(get<0>(actual.attributes())[1] ==
                get<0>(reference.attributes())[1]);
        REQUIRE(get<0>(actual.attributes())[2] ==
                get<0>(reference.attributes())[2]);
        REQUIRE(actual.normDisp == reference.normDisp);
        REQUIRE(actual.normWeight == 2 * reference.normWeight);
      }
    }

    WHEN("The pixel is blended with another pixel that has invalid depth") {
      Acc accumInvalid =
          pixel.construct(reference.attributes(), 0.f, ray_angle, stretching);
      Value actual = pixel.average(pixel.blend(accum, accumInvalid));

      THEN("It is af the pixel has not been blended") {
        REQUIRE(get<0>(actual.attributes())[0] ==
                get<0>(reference.attributes())[0]);
        REQUIRE(get<0>(actual.attributes())[1] ==
                get<0>(reference.attributes())[1]);
        REQUIRE(get<0>(actual.attributes())[2] ==
                get<0>(reference.attributes())[2]);
        REQUIRE(actual.normDisp == reference.normDisp);
        REQUIRE(actual.normWeight == reference.normWeight);
      }
    }
  }
}

SCENARIO("Reprojecting points", "[reprojectPoints]") {
  GIVEN("A camera and a depth map") {
    CameraParameters camera{42,
                            {100, 50},         // size
                            {1.f, 0.f, -1.f},  // position
                            {1.f, 2.f, -0.5f}, // orientation
                            ProjectionType::ERP,
                            {-180.f, 180.f}, // phi range
                            {-90.f, 90.f},   // theta range
                            {},
                            {},
                            {},
                            {1.f, 10.f}}; // depth range

    Mat<float> depth({50u, 100u});
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

// TODO: Add promotion rules to the linear algebra template classes so that we
// can mix uint16_t with float (or int) in the renderer
SCENARIO("Rastering meshes with 16-bit color as attribute", "[Rasterizer]") {
  GIVEN("A new rasterizer") {
    Rasterizer<Vec3w> rasterizer(AccumulatingPixel<Vec3w>{1.f, 1.f, 1.f},
                                 Vec2i{8, 4});

    WHEN("Rastering nothing") {
      THEN("The depth map is a matrix of NaN's or Inf's") {
        auto depth = rasterizer.depth();
        static_assert(is_same_v<decltype(depth), Mat<float>>);
        REQUIRE(depth.sizes() == array{4u, 8u});
        REQUIRE(none_of(begin(depth), end(depth), isfinite<float>));
      }
      THEN("The normalized disparity map is a matrix of zeroes") {
        auto normDisp = rasterizer.normDisp();
        static_assert(is_same_v<decltype(normDisp), Mat<float>>);
        REQUIRE(normDisp.sizes() == array{4u, 8u});
        REQUIRE(all_of(begin(normDisp), end(normDisp),
                       [](float x) { return x == 0.f; }));
      }
      THEN("The normalized weight (quality) map is a matrix of zeros") {
        auto normWeight = rasterizer.normWeight();
        static_assert(is_same_v<decltype(normWeight), Mat<float>>);
        REQUIRE(normWeight.sizes() == array{4u, 8u});
        REQUIRE(all_of(begin(normWeight), end(normWeight),
                       [](float x) { return x == 0.f; }));
      }
      THEN("The color map is a matrix of zero vectors") {
        auto color = rasterizer.attribute<0>();
        static_assert(is_same_v<decltype(color), Mat<Vec3w>>);
        REQUIRE(color.sizes() == array{4u, 8u});
        REQUIRE(all_of(begin(color), end(color),
                       [](Vec3w x) { return x == Vec3w{}; }));
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
        REQUIRE(depth.sizes() == array{4u, 8u});
        REQUIRE(none_of(begin(depth), end(depth), isfinite<float>));
      }
      THEN("The normalized disparity map is a matrix of zeroes") {
        auto normDisp = rasterizer.normDisp();
        cout << "normDisp: " << normDisp << endl;
        static_assert(is_same_v<decltype(normDisp), Mat<float>>);
        REQUIRE(normDisp.sizes() == array{4u, 8u});
        REQUIRE(all_of(begin(normDisp), end(normDisp),
                       [](float x) { return x == 0.f; }));
      }
      THEN("The normalized weight (quality) map is a matrix of zeros") {
        auto normWeight = rasterizer.normWeight();
        cout << "normWeight: " << normWeight << endl;
        static_assert(is_same_v<decltype(normWeight), Mat<float>>);
        REQUIRE(normWeight.sizes() == array{4u, 8u});
        REQUIRE(all_of(begin(normWeight), end(normWeight),
                       [](float x) { return x == 0.f; }));
      }
      THEN("The color map is a matrix of zero vectors") {
        auto color = rasterizer.attribute<0>();
        static_assert(is_same_v<decltype(color), Mat<Vec3w>>);
        REQUIRE(color.sizes() == array{4u, 8u});
        REQUIRE(all_of(begin(color), end(color),
                       [](Vec3w x) { return x == Vec3w{}; }));
      }
    }
    WHEN("Rastering a quad") {
      // Rectangle of 2 x 6 pixels within the 4 x 8 pixels frame
      // Depth values misused to correspond to x-values
      // Ray angles misused to correspond to y-values
      ImageVertexDescriptorList vs{{{1, 1}, 1.f, 1.f},
                                   {{7, 1}, 7.f, 1.f},
                                   {{7, 3}, 7.f, 3.f},
                                   {{1, 3}, 1.f, 3.f}};

      // Two clockwise triangles
      TriangleDescriptorList ts{{{0, 1, 2}, 6.f}, {{1, 2, 3}, 6.f}};

      // Colors correspond to x and y-values times 100
      vector<Vec3w> as{
          {100, 0, 100}, {700, 0, 100}, {700, 0, 300}, {100, 0, 300}};

      rasterizer.submit(vs, tuple{as}, ts);
      rasterizer.run();

      THEN("The depth map has known values") {
        auto depth = rasterizer.depth();
        cout << "depth: " << depth << endl;
        REQUIRE(!isfinite(depth(0, 0)));
        REQUIRE(depth(1, 1) == 1.f);
        REQUIRE(depth(1, 5) == 5.f);
        REQUIRE(depth(2, 7) == 7.f);
        REQUIRE(!isfinite(depth(3, 7)));
      }
      THEN("The normalized disparity map has known values") {
        auto normDisp = rasterizer.normDisp();
        cout << "normDisp: " << normDisp << endl;
        REQUIRE(normDisp(0, 0) == 0.f);
        REQUIRE(normDisp(1, 1) == 1.f / 1.f);
        REQUIRE(normDisp(1, 5) == 1.f / 5.f);
        REQUIRE(normDisp(2, 7) == 1.f / 7.f);
        REQUIRE(normDisp(3, 7) == 0.f);
      }
      THEN("The normalized weight (quality) map is constant within the quad "
           "and zero outside") {
        auto normWeight = rasterizer.normWeight();
        cout << "normWeight: " << normWeight << endl;
        REQUIRE(normWeight(0, 0) == 0.f);
        const auto c = normWeight(1, 1);
        REQUIRE(c > 0.f);
        REQUIRE(normWeight(1, 5) == c);
        REQUIRE(normWeight(2, 7) == c);
        REQUIRE(normWeight(3, 7) == 0.f);
      }
      THEN("The color map has known values") {
        auto color = rasterizer.attribute<0>();
        cout << "color: " << color << endl;
        REQUIRE(color(0, 0) == Vec3w{});
        REQUIRE(color(1, 1) == Vec3w{100, 0, 100});
        REQUIRE(color(1, 5) == Vec3w{100, 0, 500});
        REQUIRE(color(2, 7) == Vec3w{200, 0, 700});
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
            REQUIRE(color2(1, 1) == Vec3w{100, 0, 100});
            REQUIRE(color2(1, 5) == Vec3w{100, 0, 500});
            REQUIRE(color2(2, 7) == Vec3w{200, 0, 700});
            REQUIRE(color2(3, 7) == Vec3w{});
          }
        }
      }
    }
  }
}

SCENARIO("Rastering meshes with Vec2f as attribute", "[Rasterizer]") {
  GIVEN("A new rasterizer") {
    Rasterizer<Vec2f> rasterizer(AccumulatingPixel<Vec2f>{1.f, 1.f, 1.f},
                                 Vec2i{8, 4});

    WHEN("Rastering nothing") {
      THEN("The field map is a matrix of zero vectors") {
        auto field = rasterizer.attribute<0>();
        static_assert(is_same_v<decltype(field), Mat<Vec2f>>);
        REQUIRE(field.sizes() == array{4u, 8u});
        REQUIRE(all_of(begin(field), end(field),
                       [](Vec2f x) { return x == Vec2f{}; }));
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
        REQUIRE(field.sizes() == array{4u, 8u});
        REQUIRE(all_of(begin(field), end(field),
                       [](Vec2f x) { return x == Vec2f{}; }));
      }
    }
    WHEN("Rastering a quad") {
      // Rectangle of 2 x 6 pixels within the 4 x 8 pixels frame
      // Depth values misused to correspond to x-values
      // Ray angles misused to correspond to y-values
      ImageVertexDescriptorList vs{{{1, 1}, 1.f, 1.f},
                                   {{7, 1}, 7.f, 1.f},
                                   {{7, 3}, 7.f, 3.f},
                                   {{1, 3}, 1.f, 3.f}};

      // Two clockwise triangles
      TriangleDescriptorList ts{{{0, 1, 2}, 6.f}, {{1, 2, 3}, 6.f}};

      // Colors correspond to x and y-values times 100
      vector<Vec2f> as{
          {100.f, 100.f}, {700.f, 100.f}, {700.f, 300.f}, {100.f, 300.f}};

      rasterizer.submit(vs, tuple{as}, ts);
      rasterizer.run();

      THEN("The field map has known values") {
        auto field = rasterizer.attribute<0>();
        cout << "field: " << field << endl;
        REQUIRE(field(0, 0) == Vec2f{});
        REQUIRE(field(1, 1) == Vec2f{100.f, 100.f});
        REQUIRE(field(1, 5) == Vec2f{100.f, 500.f});
        REQUIRE(field(2, 7) == Vec2f{200.f, 700.f});
        REQUIRE(field(3, 7) == Vec2f{});

        WHEN("Rastering more meshes") {
          rasterizer.submit({}, {}, {});
          rasterizer.run();

          THEN("This is cumulative") {
            auto field2 = rasterizer.attribute<0>();
            REQUIRE(field2(0, 0) == Vec2f{});
            REQUIRE(field2(1, 1) == Vec2f{100.f, 100.f});
            REQUIRE(field2(1, 5) == Vec2f{100.f, 500.f});
            REQUIRE(field2(2, 7) == Vec2f{200.f, 700.f});
            REQUIRE(field2(3, 7) == Vec2f{});
          }
        }
      }
    }
  }
}

/*
SCENARIO("Synthesis of a depth map", "[Synthesizer]") {
  using Mat1f = TMIV::Common::Mat<float>;

  GIVEN("A synthesizer, camera and a depth map") {
    Synthesizer synthesizer{1., 1., 1.};

    CameraParameters camera{42,
                            {100, 50},         // size
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
    fill(begin(depth), end(depth), 2.f);

    WHEN("Synthesizing to the same viewpoint") {
      auto actual = synthesizer.renderDepth(depth, camera, camera);
      REQUIRE(actual.width() == 100);
      REQUIRE(actual.height() == 50);

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
*/