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

#include <TMIV/Encoder/MivEncoder.h>

TEST_CASE("Encoder::MivEncoder") {
  using TMIV::Encoder::MivEncoder;

  std::ostringstream stream;
  auto unit = std::make_unique<MivEncoder>(stream);

  SECTION("Minimal example") {
    const auto au = []() {
      auto result = TMIV::Encoder::EncoderParams{};

      static constexpr auto frameWidth = 1920;
      static constexpr auto frameHeight = 1080;
      result.vps.vps_frame_width({}, frameWidth).vps_frame_height({}, frameHeight);

      auto &atlas = result.atlas.emplace_back();
      atlas.asps.asps_frame_width(frameWidth)
          .asps_frame_height(frameHeight)
          .asps_num_ref_atlas_frame_lists_in_asps(1);
      atlas.ath.ath_type(TMIV::MivBitstream::AthType::I_TILE)
          .ath_ref_atlas_frame_list_asps_flag(true);

      [[maybe_unused]] auto &vp = result.viewParamsList.emplace_back();
      [[maybe_unused]] auto &casme = result.casps.casps_miv_extension();

      return result;
    }();

    unit->writeAccessUnit(au, false);
    unit.reset();

    REQUIRE(stream.tellp() == 145);
  }
}
