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

#include <catch2/catch.hpp>

#include <TMIV/Decoder/MivDecoder.h>

#include "FakeV3cUnitSource.h"

using Catch::Contains;
using TMIV::MivBitstream::AtlasSubBitstream;
using TMIV::MivBitstream::PtlProfileReconstructionIdc;
using TMIV::MivBitstream::PtlProfileToolsetIdc;
using TMIV::MivBitstream::SampleStreamNalHeader;
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::V3cUnit;
using TMIV::MivBitstream::V3cUnitHeader;
using TMIV::MivBitstream::VuhUnitType;

TEST_CASE("MivDecoder") {
  using TMIV::Decoder::MivDecoder;

  SECTION("Empty unit source") {
    const auto source = test::FakeV3cUnitSource{};
    auto unit = MivDecoder{source};

    REQUIRE_THROWS_WITH(unit(), Contains("No VPS"));
  }

  SECTION("The first V3C unit has to be a VPS (for this decoder)") {
    auto source = test::FakeV3cUnitSource{};

    const auto ssnh = SampleStreamNalHeader{2};
    const auto asb = AtlasSubBitstream{ssnh};
    source.units.push_back(std::make_shared<V3cUnit>(V3cUnitHeader::ad(0, {}), asb));

    auto unit = MivDecoder{source};

    REQUIRE_THROWS_WITH(unit(), Contains("No VPS"));
  }

  SECTION("The VPS needs to have the MIV extension enabled") {
    auto source = test::FakeV3cUnitSource{};

    const auto vps = V3cParameterSet{};
    source.units.push_back(std::make_shared<V3cUnit>(V3cUnitHeader::vps(), vps));

    auto unit = MivDecoder{source};

    REQUIRE_THROWS_WITH(unit(), Contains("vps_miv_extension_present_flag()"));
  }

  SECTION("The VPS needs to have matching PTL information") {
    auto source = test::FakeV3cUnitSource{};

    auto vps = V3cParameterSet{};
    vps.vps_miv_extension() = {};
    source.units.push_back(std::make_shared<V3cUnit>(V3cUnitHeader::vps(), vps));

    auto unit = MivDecoder{source};

    REQUIRE_THROWS_WITH(unit(), Contains("outside of the profile-tier-level"));
  }

  SECTION("There needs to be at least one access unit following the VPS") {
    auto source = test::FakeV3cUnitSource{};

    auto vps = V3cParameterSet{};
    vps.profile_tier_level()
        .ptl_profile_toolset_idc(PtlProfileToolsetIdc::MIV_Main)
        .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::Rec_Unconstrained);
    vps.vps_miv_extension() = {};
    source.units.push_back(std::make_shared<V3cUnit>(V3cUnitHeader::vps(), vps));

    auto unit = MivDecoder{source};

    REQUIRE_THROWS_WITH(unit(), Contains("access unit"));
  }
}
