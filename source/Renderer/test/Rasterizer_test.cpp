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

#include <TMIV/Renderer/Rasterizer.h>

using TMIV::Common::Mat;
using TMIV::Common::Vec2i;
using TMIV::Common::Vec3f;
using TMIV::Common::Vec3w;
using TMIV::Renderer::AccumulatingPixel;
using TMIV::Renderer::ImageVertexDescriptorList;
using TMIV::Renderer::Rasterizer;
using TMIV::Renderer::TriangleDescriptorList;

SCENARIO("Rastering meshes", "[Rasterizer]") {
  GIVEN("A new rasterizer") {
    Rasterizer rasterizer(AccumulatingPixel{1.F, 1.F, 1.F, 10.F}, Vec2i{8, 4});

    WHEN("Rastering nothing") {
      THEN("The field std::map is a matrix of zero vectors") {
        auto field = rasterizer.color();
        static_assert(std::is_same_v<decltype(field), Mat<Vec3f>>);
        REQUIRE(field.sizes() == std::array{size_t{4}, size_t{8}});
        REQUIRE(
            std::all_of(std::begin(field), std::end(field), [](Vec3f x) { return x == Vec3f{}; }));
      }
    }
    WHEN("Rastering some empty meshes") {
      ImageVertexDescriptorList vs;
      TriangleDescriptorList ts;
      std::vector<Vec3f> as;
      rasterizer.submit(vs, as, ts);
      rasterizer.submit(vs, as, ts);
      rasterizer.run();

      THEN("The field std::map is a matrix of zero vectors") {
        auto field = rasterizer.color();
        static_assert(std::is_same_v<decltype(field), Mat<Vec3f>>);
        REQUIRE(field.sizes() == std::array{size_t{4}, size_t{8}});
        REQUIRE(
            std::all_of(std::begin(field), std::end(field), [](Vec3f x) { return x == Vec3f{}; }));
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
      std::vector<Vec3f> as{{100.F, 100.F}, {700.F, 100.F}, {700.F, 300.F}, {100.F, 300.F}};
      rasterizer.submit(vs, as, ts);
      rasterizer.run();

      THEN("The field std::map has known values") {
        auto field = rasterizer.color();
        REQUIRE(field(0, 0) == Vec3f{});
        REQUIRE(field(1, 1).x() == Catch::Approx(150.F));
        REQUIRE(field(1, 1).y() == Catch::Approx(150.F));
        REQUIRE(field(1, 5).x() == Catch::Approx(550.F));
        REQUIRE(field(1, 5).y() == Catch::Approx(150.F));
        REQUIRE(field(2, 7) == Vec3f{});
        REQUIRE(field(3, 7) == Vec3f{});

        WHEN("Rastering more meshes") {
          rasterizer.submit({}, {}, {});
          rasterizer.run();

          THEN("This is cumulative") {
            auto field2 = rasterizer.color();
            REQUIRE(field2(0, 0) == Vec3f{});
            REQUIRE(field2(1, 1).x() == Catch::Approx(150.F));
            REQUIRE(field2(1, 1).y() == Catch::Approx(150.F));
            REQUIRE(field2(1, 5).x() == Catch::Approx(550.F));
            REQUIRE(field2(1, 5).y() == Catch::Approx(150.F));
            REQUIRE(field2(2, 7) == Vec3f{});
            REQUIRE(field2(3, 7) == Vec3f{});
          }
        }
      }
    }
  }
}
