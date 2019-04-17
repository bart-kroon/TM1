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
#include <algorithm>
#include <cmath>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::Renderer;

auto makeFullERPCamera() {
  return CameraParameters{42,
                          {10, 5},           // size
                          {1.f, 0.f, -1.f},  // position
                          {1.f, 2.f, -0.5f}, // orientation
                          ProjectionType::ERP,
                          {-180.f, 180.f}, // phi range
                          {-90.f, 90.f},   // theta range
                          {},
                          {},
                          {},
                          {1.f, 10.f}}; // depth range
}

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

  /*
  SECTION("Triangles across 180deg boundary are split") {
    const auto inVertices =
        SceneVertexDescriptorList{{{-4.f, -1.f, -1.f}, 0.f},  // Vertex 0
                                  {{-4.f, -1.f, +1.f}, 0.f},  // Vertex 1
                                  {{-4.f, +1.f, +1.f}, 0.f},  // Vertex 2
                                  {{-4.f, -2.f, +1.f}, 0.f},  // Vertex 3
                                  {{+4.f, -1.f, -1.f}, 0.f},  // Vertex 4
                                  {{+4.f, -1.f, +1.f}, 0.f},  // Vertex 5
                                  {{+4.f, +1.f, +1.f}, 0.f}}; // Vertex 6
    const auto inTriangles = TriangleDescriptorList{
        {{0, 1, 2}, 1.f},  // Triangle across 180deg boundary (one side)
        {{0, 2, 1}, 1.f},  // Triangle across 180deg boundary (other side)
        {{0, 1, 3}, 1.f},  // Triangle next to the boundary (one side)
        {{0, 3, 1}, 1.f},  // Triangle left of the boundary (other side)
        {{4, 5, 6}, 1.f},  // Triangle in front of the boundary (one side)
        {{4, 6, 5}, 1.f}}; // Triangle in front of the boundary (one side)
    const auto inAttributes =
        tuple<vector<int>>{{900, 100, 200, 300, 400, 500, 600}};

    const auto [outVertices, outTriangles, outAttributes] =
        project(inVertices, inTriangles, inAttributes, camera);

    // There are two triangles and four edges across the boundary
    REQUIRE(outVertices.size() == inVertices.size() + 4u);
    REQUIRE(get<0>(outAttributes).size() == outVertices.size());

    // The two triangles are split in three resulting in a net extra of four
    // triangles
    REQUIRE(outTriangles.size() == inTriangles.size() + 4u);

    // The 0:1:2 and 0:2:1 triangles should have been removed
    REQUIRE(
        end(inTriangles) ==
        find_if(begin(inTriangles), end(inTriangles), [](TriangleDescriptor t) {
          sort(begin(t.indices), end(t.indices));
          return t.indices[0] == 0 && t.indices[1] == 1 && t.indices[2] == 2;
        }));

    // The attributes are interpolated
    set<int> ids;
    for (auto id : get<0>(outAttributes)) {
      ids.insert(id);
    }
    REQUIRE(ids == set<int>{100, 150, 200, 300, 400, 500, 550, 600, 900});
  }
*/
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

  GIVEN("A default-constructed accumulator") {
    Acc acc;
    Pixel pixel{1.f, 1.f, 1.f};

    THEN("The attributes are zero")
    REQUIRE(std::get<0>(acc.attributes()).x() == 0.f);
    REQUIRE(std::get<0>(acc.attributes()).y() == 0.f);
    REQUIRE(std::get<0>(acc.attributes()).z() == 0.f);

    WHEN("Averaging") {
      auto val = pixel.average(acc);

      THEN("The attributes are zero") {
        REQUIRE(std::get<0>(val.attributes()).x() == 0.f);
        REQUIRE(std::get<0>(val.attributes()).y() == 0.f);
        REQUIRE(std::get<0>(val.attributes()).z() == 0.f);
      }
    }
  }

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
    auto camera = makeFullERPCamera();
    Mat<float> depth({5u, 10u});
    fill(begin(depth), end(depth), 2.f);

    WHEN("Calculating image positions") {
      auto positions = imagePositions(camera);

      THEN("The image positions should be at the pixel centers") {
        REQUIRE(positions(4, 7).x() == 7.5f);
        REQUIRE(positions(0, 0).x() == 0.5f);
        REQUIRE(positions(0, 0).y() == 0.5f);
        REQUIRE(positions(4, 9).x() == 9.5f);
        REQUIRE(positions(4, 9).y() == 4.5f);
      }

      WHEN("Reprojecting points to the same camera with valid depth "
           "values") {
        auto actual = reprojectPoints(camera, camera, positions, depth);

        THEN("The positions should not change too much") {
          REQUIRE(actual.first(4, 7).x() == Approx(7.5f));
          REQUIRE(actual.first(0, 0).x() == Approx(0.5f));
          REQUIRE(actual.first(0, 0).y() == Approx(0.5f).margin(1e-4));
          REQUIRE(actual.first(4, 9).x() == Approx(9.5f));
          REQUIRE(actual.first(4, 9).y() == Approx(4.5f));
        }

        THEN("The depth values should not change too much") {
          REQUIRE(actual.second(4, 7) == Approx(2.f));
          REQUIRE(actual.second(0, 0) == Approx(2.f));
          REQUIRE(actual.second(0, 0) == Approx(2.f));
          REQUIRE(actual.second(4, 9) == Approx(2.f));
          REQUIRE(actual.second(4, 9) == Approx(2.f));
        }
      }
    }
  }
}

SCENARIO("Rastering meshes with 16-bit color as attribute", "[Rasterizer]") {
  GIVEN("A new rasterizer") {
    AccumulatingPixel<Vec3w> pixel{1.f, 1.f, 1.f};
    Rasterizer<Vec3w> rasterizer(pixel, Vec2i{8, 4});

    WHEN("Rastering nothing") {
      THEN("The depth map is a matrix of NaN's or Inf's") {
        auto depth = rasterizer.depth();
        static_assert(is_same_v<decltype(depth), Mat<float>>);
        REQUIRE(depth.sizes() == array{4u, 8u});
        REQUIRE(none_of(begin(depth), end(depth),
                        [](float x) { return isfinite(x); }));
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
        REQUIRE(none_of(begin(depth), end(depth),
                        [](float x) { return isfinite(x); }));
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
    WHEN("Rastering a quad") {
      // Rectangle of 2 x 6 pixels within the 4 x 8 pixels frame
      // Depth values misused to correspond to x-values
      // Ray angles misused to correspond to y-values
      ImageVertexDescriptorList vs{{{1, 1}, 1.f, 1.f},
                                   {{7, 1}, 7.f, 1.f},
                                   {{7, 3}, 7.f, 3.f},
                                   {{1, 3}, 1.f, 3.f}};

      // Two clockwise triangles
      TriangleDescriptorList ts{{{0, 1, 2}, 3.f}, {{0, 2, 3}, 5.f}};

      // Colors correspond to x and y-values times 100
      vector<Vec3w> as{
          {100, 0, 100}, {700, 0, 100}, {700, 0, 300}, {100, 0, 300}};

      rasterizer.submit(vs, tuple{as}, ts);
      rasterizer.run();

      THEN("The depth map has known values") {
        auto depth = rasterizer.depth();
        REQUIRE(!isfinite(depth(0, 0)));
        REQUIRE(depth(1, 1) == 1.f / (11.f / 12.f + 1.f / 12.f / 7.f));
        REQUIRE(depth(1, 5) == 1.f / (3.f / 12.f + 9.f / 12.f / 7.f));
        REQUIRE(!isfinite(depth(2, 7)));
        REQUIRE(!isfinite(depth(3, 7)));
      }
      THEN("The normalized disparity map has known values") {
        auto normDisp = rasterizer.normDisp();
        REQUIRE(normDisp(0, 0) == 0.f);
        REQUIRE(normDisp(1, 1) == 11.f / 12.f + 1.f / 12.f / 7.f);
        REQUIRE(normDisp(1, 5) == 3.f / 12.f + 9.f / 12.f / 7.f);
        REQUIRE(normDisp(2, 7) == 0.f);
        REQUIRE(normDisp(3, 7) == 0.f);
      }
      THEN("The normalized weight (quality) has known values for except for "
           "points that intersect triangle edges") {
        auto normWeight = rasterizer.normWeight();

        // The average ray angle of all three vertices is used for all
        // points in a triangle Note that ray angles are artifical examples.
        // Typical values would be <0.1 rad.
        const float rayAngle1 = 1.f * (2.f / 3.f) + 3.f * (1.f / 3.f);
        const float rayAngle2 = 1.f * (1.f / 3.f) + 3.f * (2.f / 3.f);
        const float w_rayAngle1 = pixel.rayAngleWeight(rayAngle1);
        const float w_rayAngle2 = pixel.rayAngleWeight(rayAngle2);

        // The same stretching weight is used for all points in a triangle.
        // The stretching is the ratio of original and synthesized areas.
        const float w_stretching1 = pixel.rayAngleWeight(6.f / 3.f);
        const float w_stretching2 = pixel.rayAngleWeight(6.f / 5.f);

        REQUIRE(normWeight(0, 0) == 0.f);
        REQUIRE(normWeight(1, 1) == Approx(w_rayAngle2 * w_stretching2));
        REQUIRE(normWeight(1, 5) == Approx(w_rayAngle1 * w_stretching1));
        REQUIRE(normWeight(2, 7) == 0.f);
        REQUIRE(normWeight(3, 7) == 0.f);
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
      TriangleDescriptorList ts{{{0, 1, 2}, 6.f}, {{0, 2, 3}, 6.f}};

      // Colors correspond to x and y-values times 100
      vector<Vec2f> as{
          {100.f, 100.f}, {700.f, 100.f}, {700.f, 300.f}, {100.f, 300.f}};

      rasterizer.submit(vs, tuple{as}, ts);
      rasterizer.run();

      THEN("The field map has known values") {
        auto field = rasterizer.attribute<0>();
        REQUIRE(field(0, 0) == Vec2f{});
        REQUIRE(field(1, 1).x() == Approx(150.f));
        REQUIRE(field(1, 1).y() == Approx(150.f));
        REQUIRE(field(1, 5).x() == Approx(550.f));
        REQUIRE(field(1, 5).y() == Approx(150.f));
        REQUIRE(field(2, 7) == Vec2f{});
        REQUIRE(field(3, 7) == Vec2f{});

        WHEN("Rastering more meshes") {
          rasterizer.submit({}, {}, {});
          rasterizer.run();

          THEN("This is cumulative") {
            auto field2 = rasterizer.attribute<0>();
            REQUIRE(field2(0, 0) == Vec2f{});
            REQUIRE(field2(1, 1).x() == Approx(150.f));
            REQUIRE(field2(1, 1).y() == Approx(150.f));
            REQUIRE(field2(1, 5).x() == Approx(550.f));
            REQUIRE(field2(1, 5).y() == Approx(150.f));
            REQUIRE(field2(2, 7) == Vec2f{});
            REQUIRE(field2(3, 7) == Vec2f{});
          }
        }
      }
    }
  }
}

SCENARIO("Synthesis of a depth map", "[Synthesizer]") {
  using Mat1f = TMIV::Common::Mat<float>;

  GIVEN("A synthesizer, camera and a depth map") {
    Synthesizer synthesizer{1., 1., 1.};
    auto camera = makeFullERPCamera();

    Mat1f depth({unsigned(camera.size.y()), unsigned(camera.size.x())});
    fill(begin(depth), end(depth), 2.f);

    WHEN("Synthesizing to the same viewpoint") {
      auto actual = synthesizer.renderDepth(depth, camera, camera);
      REQUIRE(actual.width() == 10);
      REQUIRE(actual.height() == 5);

      THEN("The output depth should match the input depth (EXCEPT FOR THE "
           "RIGHT-MOST COLUMN)") {
        for (auto i = 0u; i != depth.height(); ++i) {
          for (auto j = 0u; j + 1 < depth.width(); ++j) {
            REQUIRE(actual(i, j) == Approx(2.f));
          }
        }
      }

      /*
  THEN("The last columns should also match the input depth\n(BUT IT "
       "CURRENTLY DOES NOT)") {
    // The ERP projector should split and cull triangles, but this has
    // not been implemented. Until then this test case serves as a
    // reminder of this unfinished work.
    for (auto i = 0u; i != depth.height(); ++i) {
      const int j = depth.width() - 1;
      REQUIRE(actual(i, j) == Approx(2.f));
    }
  }
      */
    }
  }
}
