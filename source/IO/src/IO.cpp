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

#include <TMIV/IO/IO.h>

#include <TMIV/Common/format.h>

#include "DependencyInjector.h"
#include "Filesystem.h"

using namespace std::string_literals;

namespace TMIV::IO {
auto videoComponentName(MivBitstream::VuhUnitType vuhUnitType,
                        MivBitstream::AiAttributeTypeId attrTypeId) -> std::string {
  switch (vuhUnitType) {
  case MivBitstream::VuhUnitType::V3C_OVD:
    return "Occupancy"s;
  case MivBitstream::VuhUnitType::V3C_GVD:
    return "Geometry"s;
  case MivBitstream::VuhUnitType::V3C_PVD:
    return "Packed"s;
  case MivBitstream::VuhUnitType::V3C_AVD:
    switch (attrTypeId) {
    case MivBitstream::AiAttributeTypeId::ATTR_TEXTURE:
      return "Texture"s;
    case MivBitstream::AiAttributeTypeId::ATTR_MATERIAL_ID:
      return "MaterialId"s;
    case MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY:
      return "Transparency"s;
    case MivBitstream::AiAttributeTypeId::ATTR_REFLECTANCE:
      return "Reflectance"s;
    case MivBitstream::AiAttributeTypeId::ATTR_NORMAL:
      return "Normal"s;
    default:
      UNREACHABLE;
    }
  default:
    UNREACHABLE;
  }
}

auto videoFormatString(Common::ColorFormat colorFormat, uint32_t bitDepth) -> std::string {
  std::string colorFormatString;

  switch (colorFormat) {
  case Common::ColorFormat::YUV400:
    colorFormatString = "gray"s;
    break;
  case Common::ColorFormat::YUV420:
    colorFormatString = "yuv420p"s;
    break;
  case Common::ColorFormat::YUV444:
    colorFormatString = "yuv444p"s;
    break;
  default:
    UNREACHABLE;
  }

  if (bitDepth == 8) {
    return colorFormatString;
  }
  if (bitDepth < 8) {
    return TMIV_FMT::format("{}{}", colorFormatString, bitDepth);
  }
  return TMIV_FMT::format("{}{}le", colorFormatString, bitDepth);
}
} // namespace TMIV::IO
