/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#include <TMIV/PtlChecker/AbstractChecker.h>

namespace test {
class FakeChecker : public TMIV::PtlChecker::AbstractChecker {
public:
  void replaceLogger(Logger /* value */) override { UNREACHABLE; }

  void checkVuh(const TMIV::MivBitstream::V3cUnitHeader &vuh) override {
    if (vuh.vuh_unit_type() != TMIV::MivBitstream::VuhUnitType::V3C_VPS) {
      REQUIRE(activeVps.has_value());
    }

    ++checkVuh_callCount;
    lastVuh = vuh;
  }

  void checkNuh(const TMIV::MivBitstream::NalUnitHeader & /* nuh */) override {
    ++checkAndActivateNuh_callCount;
  }

  void checkAndActivateVps(const TMIV::MivBitstream::V3cParameterSet &vps) override {
    ++checkAndActivateVps_callCount;
    activeVps = vps;
  }

  void activateCasps(
      const TMIV::MivBitstream::CommonAtlasSequenceParameterSetRBSP & /* casps */) override {}

  void checkAsps(TMIV::MivBitstream::AtlasId /* atlasId */,
                 const TMIV::MivBitstream::AtlasSequenceParameterSetRBSP & /* asps */) override {
    REQUIRE(activeVps.has_value());

    ++checkAndActivateAsps_callCount;
  }

  void checkAfps(const TMIV::MivBitstream::AtlasFrameParameterSetRBSP & /* afps */) override {
    ++checkAfps_callCount;
  }

  void checkAtl(const TMIV::MivBitstream::NalUnitHeader & /* nuh */,
                const TMIV::MivBitstream::AtlasTileLayerRBSP & /* atl */) override {
    ++checkAtl_callCount;
  }

  void checkCaf(const TMIV::MivBitstream::NalUnitHeader & /* nuh */,
                const TMIV::MivBitstream::CommonAtlasFrameRBSP & /* atl */) override {
    ++checkCaf_callCount;
  }

  void checkVideoFrame(TMIV::MivBitstream::VuhUnitType /* vut */,
                       const TMIV::MivBitstream::AtlasSequenceParameterSetRBSP & /*asps */,
                       const TMIV::Common::Frame<> &frame) override {
    REQUIRE(!frame.empty());
    REQUIRE(activeVps.has_value());

    ++checkVideoFrame_callCount;
  }

  void checkV3cFrame([[maybe_unused]] const TMIV::MivBitstream::AccessUnit &frame) override {
    ++checkV3cFrame_callCount;
  }

  size_t checkVuh_callCount{};
  size_t checkAndActivateNuh_callCount{};
  size_t checkAndActivateVps_callCount{};
  size_t checkAndActivateAsps_callCount{};
  size_t checkAfps_callCount{};
  size_t checkAtl_callCount{};
  size_t checkCaf_callCount{};
  size_t checkVideoFrame_callCount{};
  size_t checkV3cFrame_callCount{};

  std::optional<TMIV::MivBitstream::V3cUnitHeader> lastVuh;
  std::optional<TMIV::MivBitstream::V3cParameterSet> activeVps;
};
} // namespace test
