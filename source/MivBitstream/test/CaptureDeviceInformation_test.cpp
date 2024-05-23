/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#include <TMIV/MivBitstream/CaptureDeviceInformation.h>

namespace TMIV::MivBitstream {
TEST_CASE("caputre_device_information", "[Common Atlas Frame RBSP]") {
  auto unit = CaptureDeviceInformation{};

  SECTION("Default constructor") {
    REQUIRE(toString(unit) == R"(cdi_device_model_count_minus1=0
cdi_device_model_id[0]=0
cdi_device_class_id[0]=0
)");

    bitCodingTest(unit, 8);
  }

  SECTION("Example 1") {
    unit.cdi_device_model_count_minus1(63);
    for (uint16_t m = 0; m <= unit.cdi_device_model_count_minus1(); m++) {
      unit.cdi_device_model_id(m, static_cast<uint8_t>(m));
      unit.cdi_device_class_id(m, static_cast<uint8_t>(m));
      if (unit.cdi_device_class_id(m) != 0) {
        unit.cdi_sensor_count_minus1(m, m);
        for (uint16_t s = 0; s <= unit.cdi_sensor_count_minus1(m); s++) {
          unit.cdi_sensor_component_id(m, s, static_cast<uint8_t>(std::min(uint16_t{31}, s)));
        }
        unit.cdi_intra_sensor_parallax_flag(m, m % 2 == 0);
        unit.cdi_light_source_count(m, m);
        unit.cdi_infrared_image_present_flag(m, m % 2 == 0);
        unit.cdi_depth_confidence_present_flag(m, m % 2 == 0);
        if (unit.cdi_depth_confidence_present_flag(m)) {
          for (uint16_t s = 0; s <= unit.cdi_sensor_count_minus1(m); s++) {
            if (unit.cdi_sensor_component_id(m, s) == 0) {
              unit.cdi_depth_confidence_flag(m, s, (m % 2 == 0));
            }
          }
        }
      }
    }

    bitCodingTest(unit, 12786);
  }
}

} // namespace TMIV::MivBitstream
