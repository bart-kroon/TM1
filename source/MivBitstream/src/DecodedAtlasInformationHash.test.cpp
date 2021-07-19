/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#include <TMIV/MivBitstream/DecodedAtlasInformationHash.h>

namespace TMIV::MivBitstream {
TEST_CASE("decoded_atlas_information_hash", "[Decoded Atlas Information Hash SEI payload syntax]") {
  SECTION("Default constructor") {
    const DecodedAtlasInformationHash unit{};
    REQUIRE(toString(unit) == R"(daih_cancel_flag=true
)");
    REQUIRE(bitCodingTest(unit, 1));
  }

  SECTION("md5 Hash Type") {
    DecodedAtlasHash dah{};
    for (uint8_t i = 0; i < 16; i++) {
      dah.daih_atlas_md5(i, 17 * i);
    }

    DecodedAtlasInformationHash unit{};
    unit.daih_cancel_flag(false)
        .daih_persistence_flag(false)
        .daih_hash_type(0)
        .daih_decoded_high_level_hash_present_flag(false)
        .daih_decoded_atlas_hash_present_flag(true)
        .daih_decoded_atlas_b2p_hash_present_flag(false)
        .daih_decoded_atlas_tiles_hash_present_flag(false)
        .daih_decoded_atlas_tiles_b2p_hash_present_flag(false)
        .decoded_atlas_hash(dah);

    REQUIRE(toString(unit) == R"(daih_cancel_flag=false
daih_persistence_flag=false
daih_hash_type=0
daih_decoded_high_level_hash_present_flag=false
daih_decoded_atlas_hash_present_flag=true
daih_decoded_atlas_b2p_hash_present_flag=false
daih_decoded_atlas_tiles_hash_present_flag=false
daih_decoded_atlas_tiles_b2p_hash_present_flag=false
daih_atlas_md5=00112233445566778899aabbccddeeff
)");
    REQUIRE(bitCodingTest(unit, 144));
  }

  SECTION("CRC Hash Type") {
    DecodedAtlasHash dah{};
    dah.daih_atlas_crc(0x1234);

    DecodedAtlasInformationHash unit{};
    unit.daih_cancel_flag(false)
        .daih_persistence_flag(false)
        .daih_hash_type(1)
        .daih_decoded_high_level_hash_present_flag(false)
        .daih_decoded_atlas_hash_present_flag(true)
        .daih_decoded_atlas_b2p_hash_present_flag(false)
        .daih_decoded_atlas_tiles_hash_present_flag(false)
        .daih_decoded_atlas_tiles_b2p_hash_present_flag(false)
        .decoded_atlas_hash(dah);

    REQUIRE(toString(unit) == R"(daih_cancel_flag=false
daih_persistence_flag=false
daih_hash_type=1
daih_decoded_high_level_hash_present_flag=false
daih_decoded_atlas_hash_present_flag=true
daih_decoded_atlas_b2p_hash_present_flag=false
daih_decoded_atlas_tiles_hash_present_flag=false
daih_decoded_atlas_tiles_b2p_hash_present_flag=false
daih_atlas_crc=1234
)");
    REQUIRE(bitCodingTest(unit, 32));
  }

  SECTION("CheckSum Hash Type") {
    DecodedAtlasHash dah{};
    dah.daih_atlas_checksum(0x12345678);

    DecodedAtlasInformationHash unit{};
    unit.daih_cancel_flag(false)
        .daih_persistence_flag(false)
        .daih_hash_type(2)
        .daih_decoded_high_level_hash_present_flag(false)
        .daih_decoded_atlas_hash_present_flag(true)
        .daih_decoded_atlas_b2p_hash_present_flag(false)
        .daih_decoded_atlas_tiles_hash_present_flag(false)
        .daih_decoded_atlas_tiles_b2p_hash_present_flag(false)
        .decoded_atlas_hash(dah);

    REQUIRE(toString(unit) == R"(daih_cancel_flag=false
daih_persistence_flag=false
daih_hash_type=2
daih_decoded_high_level_hash_present_flag=false
daih_decoded_atlas_hash_present_flag=true
daih_decoded_atlas_b2p_hash_present_flag=false
daih_decoded_atlas_tiles_hash_present_flag=false
daih_decoded_atlas_tiles_b2p_hash_present_flag=false
daih_atlas_checksum=12345678
)");
    REQUIRE(bitCodingTest(unit, 48));
  }
}
} // namespace TMIV::MivBitstream
