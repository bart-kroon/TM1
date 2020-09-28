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

#include <TMIV/MivBitstream/PackedIndependentRegions.h>

namespace TMIV::MivBitstream {
TEST_CASE("packed_independent_regions", "[Packed Independent Regions SEI payload syntax]") {
  SECTION("Default Constructor") {
    const PackedIndependentRegions unit{};
    REQUIRE(toString(unit) == R"(pir_num_packed_frames_minus1=0
)");
    const std::size_t expected_number_of_bits = 5; // pir_num_packed_frames_minus1
    REQUIRE(bitCodingTest(unit, expected_number_of_bits));
  }
  SECTION("Frames with zero regions") {
    PackedIndependentRegions unit{};
    const std::size_t number_of_frames = 5;
    unit.pir_num_packed_frames_minus1(number_of_frames);
    for (std::size_t frame = 0; frame < number_of_frames; ++frame) {
      // TODO you could invert this
      unit.pir_packed_frame_id(frame, frame);
      const auto k = unit.pir_packed_frame_id(frame);
      unit.pir_description_type_idc(k, frame + 1);
    }
  }
}
} // namespace TMIV::MivBitstream
