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

#include "test.h"

TEST_CASE("videoComponentName") {
  using TMIV::IO::videoComponentName;
  using ATI = TMIV::MivBitstream::AiAttributeTypeId;
  using VUT = TMIV::MivBitstream::VuhUnitType;

  CHECK(videoComponentName(VUT::V3C_OVD) == "Occupancy"s);
  CHECK(videoComponentName(VUT::V3C_GVD) == "Geometry"s);
  CHECK(videoComponentName(VUT::V3C_PVD) == "Packed"s);

  CHECK(videoComponentName(VUT::V3C_AVD, ATI::ATTR_TEXTURE) == "Texture"s);
  CHECK(videoComponentName(VUT::V3C_AVD, ATI::ATTR_MATERIAL_ID) == "MaterialId"s);
  CHECK(videoComponentName(VUT::V3C_AVD, ATI::ATTR_TRANSPARENCY) == "Transparency"s);
  CHECK(videoComponentName(VUT::V3C_AVD, ATI::ATTR_REFLECTANCE) == "Reflectance"s);
  CHECK(videoComponentName(VUT::V3C_AVD, ATI::ATTR_NORMAL) == "Normal"s);
}

TEST_CASE("TMIV::IO::videoFormatString") {
  using TMIV::IO::videoFormatString;
  using CF = TMIV::Common::ColorFormat;

  SECTION("videoFormatString(colorFormat, bitDepth)") {
    CHECK(videoFormatString(CF::YUV400, 1) == "gray1");
    CHECK(videoFormatString(CF::YUV400, 2) == "gray2");
    CHECK(videoFormatString(CF::YUV400, 7) == "gray7");
    CHECK(videoFormatString(CF::YUV400, 8) == "gray");
    CHECK(videoFormatString(CF::YUV400, 9) == "gray9le");
    CHECK(videoFormatString(CF::YUV400, 16) == "gray16le");
    CHECK(videoFormatString(CF::YUV400, 17) == "gray17le");
    CHECK(videoFormatString(CF::YUV400, 32) == "gray32le");

    CHECK(videoFormatString(CF::YUV420, 1) == "yuv420p1");
    CHECK(videoFormatString(CF::YUV420, 2) == "yuv420p2");
    CHECK(videoFormatString(CF::YUV420, 7) == "yuv420p7");
    CHECK(videoFormatString(CF::YUV420, 8) == "yuv420p");
    CHECK(videoFormatString(CF::YUV420, 9) == "yuv420p9le");
    CHECK(videoFormatString(CF::YUV420, 16) == "yuv420p16le");
    CHECK(videoFormatString(CF::YUV420, 17) == "yuv420p17le");
    CHECK(videoFormatString(CF::YUV420, 32) == "yuv420p32le");

    CHECK(videoFormatString(CF::YUV444, 1) == "yuv444p1");
    CHECK(videoFormatString(CF::YUV444, 2) == "yuv444p2");
    CHECK(videoFormatString(CF::YUV444, 7) == "yuv444p7");
    CHECK(videoFormatString(CF::YUV444, 8) == "yuv444p");
    CHECK(videoFormatString(CF::YUV444, 9) == "yuv444p9le");
    CHECK(videoFormatString(CF::YUV444, 16) == "yuv444p16le");
    CHECK(videoFormatString(CF::YUV444, 17) == "yuv444p17le");
    CHECK(videoFormatString(CF::YUV444, 32) == "yuv444p32le");
  }

  SECTION("videoFormatString(frame)") {
    CHECK(videoFormatString(test::frame({4, 4}, CF::YUV444, 7)) == "yuv444p7");
  }
}
