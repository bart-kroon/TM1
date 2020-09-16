/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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

#include "test.h"

#include <TMIV/MivBitstream/SeiRBSP.h>

namespace TMIV::MivBitstream {
TEST_CASE("PayloadType", "[Supplemental Enhancement Information RBSP]") {
  REQUIRE(toString(PayloadType::viewing_space_handling) == "viewing_space_handling");
  REQUIRE(toString(PayloadType::geometry_upscaling_parameters) == "geometry_upscaling_parameters");
  REQUIRE(toString(PayloadType(42)) == "reserved_sei_message (42)");
}

TEST_CASE("sei_message", "[Supplemental Enhancement Information RBSP]") {
  SECTION("Default Constructor") {
    const auto message = SeiMessage{};
    REQUIRE(toString(message) == R"(payloadType=buffering_period
payloadSize=0
)");
    REQUIRE(byteCodingTest(message, 2));
  }

  SECTION("Time Code") {
    const auto message = SeiMessage{PayloadType::time_code, "Tick tock"};
    REQUIRE(toString(message) == R"(payloadType=time_code
payloadSize=9
)");
    REQUIRE(byteCodingTest(message, 11));
  }

  SECTION("Atlas Object Association") {
    const auto message = SeiMessage{PayloadType::atlas_object_association, "My Atlas"};
    REQUIRE(toString(message) == R"(payloadType=atlas_object_association
payloadSize=8
)");
    REQUIRE(byteCodingTest(message, 10));
  }
}

TEST_CASE("sei_rbsp", "[Supplemental Enhancement Information RBSP]") {
  auto x = SeiRBSP{};

  REQUIRE(toString(x).empty());

  SECTION("Example 1") {
    x.messages().emplace_back();
    x.messages().emplace_back(PayloadType::sei_manifest, "Manifest");

    REQUIRE(toString(x) == R"(payloadType=buffering_period
payloadSize=0
payloadType=sei_manifest
payloadSize=8
)");
    REQUIRE(byteCodingTest(x, 13));
  }

  SECTION("Example 2") {
    x.messages().emplace_back(PayloadType::filler_payload, std::string(1000, 'x'));
    x.messages().emplace_back(PayloadType::filler_payload, std::string(254, 'a'));
    x.messages().emplace_back(PayloadType::filler_payload, std::string(255, 'b'));
    x.messages().emplace_back(PayloadType::filler_payload, std::string(256, 'c'));
    x.messages().emplace_back(PayloadType::filler_payload, std::string(257, 'd'));
    x.messages().emplace_back(PayloadType::user_data_unregistered, "Unregistered");
    x.messages().emplace_back(PayloadType::atlas_object_association, std::string(7, 'e'));

    REQUIRE(toString(x) == R"(payloadType=filler_payload
payloadSize=1000
payloadType=filler_payload
payloadSize=254
payloadType=filler_payload
payloadSize=255
payloadType=filler_payload
payloadSize=256
payloadType=filler_payload
payloadSize=257
payloadType=user_data_unregistered
payloadSize=12
payloadType=atlas_object_association
payloadSize=7
)");
    const std::size_t where_do_these_atlas_bytes_come_from = 6;
    const std::size_t where_do_these_bytes_come_from = 6 + 2;
    const std::size_t expected_number_of_bytes = 13 + 1000 + 254 + 255 + 256 + 257 + 12 +
                                                 (7 + where_do_these_atlas_bytes_come_from) +
                                                 where_do_these_bytes_come_from;
    REQUIRE(byteCodingTest(x, expected_number_of_bytes));
  }
}
} // namespace TMIV::MivBitstream
