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

using Catch::Matchers::Contains;

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Common.h>
#include <TMIV/Common/Json.h>
#include <TMIV/Common/LinAlg.h>

#include <iostream>
#include <sstream>

using namespace std;

namespace TMIV::Common {

TEST_CASE("Array, Vector, Matrix, LinAlg") {
  const float EPS = 1e-5F;

  heap::Array<3, float> A({3, 5, 10});
  Mat3x3f m1 = {1.F, 2.F, 3.F, 4.F, 5.F, 6.F, 7.F, 8.F, -9.F};
  Mat<float> m2({4, 3}, {1.F, 2.F, 3.F, 4.F, 5.F, 6.F, 7.F, 8.F, 9.F, 10.F, 11.F, 12.F});
  Vec3f v1({1.F, 2.F, 3.F});
  Vec3f v2({3.F, -1.F, 6.F});

  SECTION("Array")
  REQUIRE(A.size() == 150);

  SECTION("Vector norm")
  REQUIRE(fabs(norm(v1) - 3.7416573F) < EPS);

  SECTION("Unit vector")
  REQUIRE(fabs(norm(unit(v1)) - 1.F) < EPS);

  SECTION("Vector dot product")
  REQUIRE(fabs(dot(v1, v1) - 14.F) < EPS);

  SECTION("Vector cross product")
  REQUIRE(norm_inf(cross(v1, v2) - Vec3f({15.F, 3.F, -7.F})) < EPS);

  SECTION("Matrix trace")
  REQUIRE(fabs(trace(m1) - (-3.F)) < EPS);

  SECTION("Matrix transpose")
  REQUIRE(norm_inf(transpose(m2) - Mat<float>({3, 4}, {1.F, 4.F, 7.F, 10.F, 2.F, 5.F, 8.F, 11.F,
                                                       3.F, 6.F, 9.F, 12.F})) < EPS);

  SECTION("Matrix / Vector product")
  REQUIRE(norm_inf((m1 * v1) - Vec3f({14.F, 32.F, -4.F})) < EPS);

  SECTION("Matrix / Matrix product")
  REQUIRE(norm_inf((m2 * m1) - Mat<float>({4, 3}, {30.F, 36.F, -12.F, 66.F, 81.F, -12.F, 102.F,
                                                   126.F, -12.F, 138.F, 171.F, -12.F})) < EPS);

  SECTION("Matrix inverse")
  REQUIRE(norm_inf((m1 * inv(m1)) - Mat3x3f::eye()) < EPS);

  SECTION("Linear system (right)")
  REQUIRE(norm_inf(mrdivide(m2, m1) - Mat<float>({4, 3}, {1.F, 0.F, 0.F, 0.F, 1.F, 0.F, -1.F, 2.F,
                                                          0.F, -2.F, 3.F, 0.F})) < EPS);

  SECTION("Linear system (left)")
  REQUIRE(norm_inf(mldivide(m1, v1) - Vec3f({-1.F, 2.F, 0.F}) / 3.F) < EPS);

  SECTION("Matrix determinant")
  REQUIRE(fabs(det(m1) - 54.F) < EPS);

  fill(m1.diag_begin(), m1.diag_end(), 0.F);
  SECTION("Matrix iterator")
  REQUIRE(fabs(trace(m1)) < EPS);
}

TEST_CASE("Reading a Json", "[Json]") {
  istringstream stream{R"({ "alpha": true, "beta": false })"};
  auto json = Json{stream};
  SECTION("Read booleans") {
    REQUIRE(json.require("alpha").asBool());
    REQUIRE(!json.require("beta").asBool());
    REQUIRE(json.optional("alpha"));
    REQUIRE(!json.optional("beta"));
    REQUIRE(json.optional("beta").type() == Json::Type::boolean);
    REQUIRE(!json.optional("gamma"));
    REQUIRE(json.optional("gamma").type() == Json::Type::null);
  }
}

namespace {
class FakeApplication : public Application {
public:
  using Application::Application;
  using Application::json;

  void run() override {}
};
} // namespace

TEST_CASE("Parsing the command-line", "[Application]") {
  SECTION("Empty command-line returns usage instructions") {
    try {
      FakeApplication app{"Fake", {"command"}};
      REQUIRE(false);
    } catch (runtime_error &e) {
      REQUIRE_THAT(e.what(), Contains("Usage"));
      REQUIRE_THAT(e.what(), Contains("Fake"));
    }
  }

  SECTION("Specifying parameters with -p KEY VALUE") {
    FakeApplication app{"Fake", {"command", "-p", "Color", "green", "-p", "Shape", "circular"}};
    REQUIRE(app.json().require("Color").asString() == "green");
    REQUIRE(app.json().require("Shape").asString() == "circular");
  }

  SECTION("Right has preference over left") {
    FakeApplication app{"Fake", {"command", "-p", "Color", "green", "-p", "Color", "red"}};
    REQUIRE(app.json().require("Color").asString() == "red");
  }

  SECTION("Load a Json") {
    FakeApplication app{"Fake", {"command", "-c", "Common/test/CommonTest.json"}};
    REQUIRE(app.json().require("intraPeriod").asInt() == 32);
  }

  SECTION("Load a Json and add a parameters") {
    FakeApplication app{
        "Fake", {"command", "-c", "Common/test/CommonTest.json", "-p", "continent", "Africa"}};
    REQUIRE(app.json().require("continent").asString() == "Africa");
  }

  SECTION("Load a Json and override a parameter") {
    FakeApplication app{"Fake",
                        {"command", "-c", "Common/test/CommonTest.json", "-p", "intraPeriod", "8"}};
    REQUIRE(app.json().require("intraPeriod").asInt() == 8);
  }

  SECTION("Load a Json, override a parameter and add a parameter") {
    FakeApplication app{"Fake",
                        {"command", "-c", "Common/test/CommonTest.json", "-p", "intraPeriod", "8",
                         "-p", "continent", "Africa"}};
    REQUIRE(app.json().require("intraPeriod").asInt() == 8);
    REQUIRE(app.json().require("continent").asString() == "Africa");
  }
}

TEST_CASE("Converting floating point to integer") {
  REQUIRE(ifloor(-2.5F) == -3);
  REQUIRE(ifloor(0.F) == 0);
  REQUIRE(ifloor(1000000.9F) == 1000000);
  REQUIRE(ifloor(1000001.0F) == 1000001);
  REQUIRE(ifloor(1000001.1F) == 1000001);

  REQUIRE(iceil(-2.5F) == -2);
  REQUIRE(iceil(0.F) == 0);
  REQUIRE(iceil(1000000.9F) == 1000001);
  REQUIRE(iceil(1000001.0F) == 1000001);
  REQUIRE(iceil(1000001.1F) == 1000002);
}
} // namespace TMIV::Common
