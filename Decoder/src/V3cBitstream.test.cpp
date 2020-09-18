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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Bytestream.h>
#include <TMIV/Decoder/MivDecoder.h>
#include <TMIV/MivBitstream/MivDecoderMode.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <filesystem>
#include <fstream>

auto dumpV3cUnitPayload(std::streampos position, const MivBitstream::SampleStreamV3cUnit &ssvu,
                        const MivBitstream::V3cUnitHeader &vuh) {
  std::ostringstream path;
  path << "v3c_unit_" << position << '_' << vuh.vuh_unit_type();
  if (vuh.vuh_unit_type() != VuhUnitType::V3C_VPS) {
    path << '_' << int{vuh.vuh_v3c_parameter_set_id()} << '_' << int{vuh.vuh_atlas_id()};
  }
  if (vuh.vuh_unit_type() == VuhUnitType::V3C_AVD) {
    path << '_' << int{vuh.vuh_attribute_index()} << '_'
         << int{vuh.vuh_attribute_partition_index()};
  }
  if (vuh.vuh_unit_type() == VuhUnitType::V3C_AVD || vuh.vuh_unit_type() == VuhUnitType::V3C_GVD) {
    path << '_' << int{vuh.vuh_map_index()} << '_' << std::boolalpha
         << vuh.vuh_auxiliary_video_flag();
  }
  if (vuh.vuh_unit_type() == MivBitstream::VuhUnitType::V3C_VPS ||
      vuh.vuh_unit_type() == MivBitstream::VuhUnitType::V3C_AD) {
    path << ".bit";
  } else {
    path << ".mp4";
  }

  std::ofstream file{path.str(), std::ios::binary};
  auto payload = ssvu.ssvu_v3c_unit();
  payload.erase(payload.begin(), payload.begin() + 4);
  file.write(payload.data(), payload.size());
}

void demultiplex(std::istream &stream) {
  stream.seekg(0, std::ios::end);
  const auto filesize = stream.tellg();
  std::cout << "[ 0 ]: File size is " << filesize << " bytes\n";
  stream.seekg(0);

  std::cout << "[ " << stream.tellg() << " ]: ";
  const auto ssvh = MivBitstream::SampleStreamV3cHeader::decodeFrom(stream);
  std::cout << ssvh;

  auto vpses = std::vector<MivBitstream::V3cParameterSet>{};

  while (stream.tellg() != filesize) {
    const auto position = stream.tellg();
    std::cout << "[ " << position << " ]: ";
    const auto ssvu = MivBitstream::SampleStreamV3cUnit::decodeFrom(stream, ssvh);
    std::cout << ssvu;

    std::istringstream substream{ssvu.ssvu_v3c_unit()};
    const auto vuh = MivBitstream::V3cUnitHeader::decodeFrom(substream, vpses);
    std::cout << vuh;

    dumpV3cUnitPayload(position, ssvu, vuh);

    const auto vp = V3cPayload::decodeFrom(substream, vuh);
    std::cout << vp;

    if (const auto *vps = std::get_if<MivBitstream::V3cParameterSet>(&vp.payload());
        vps != nullptr) {
      while (vps->vps_v3c_parameter_set_id() >= vpses.size()) {
        vpses.emplace_back();
      }
      std::cout << "vpses[" << int{vps->vps_v3c_parameter_set_id()} << "] := vps\n";
      vpses[vps->vps_v3c_parameter_set_id()] = *vps;
    }
  }

  std::cout << "[ " << stream.tellg() << " ].\n";
}

auto testDataDir() { return filesystem::path(__FILE__).parent_path().parent_path() / "test"; }

const auto testBitstreams =
    std::array{testDataDir() / "longdress_1frame_vpcc_ctc" / "longdress_vox10_GOF0.bin"};

TEST_CASE("Demultiplex", "[V3C bitstream]") {
  TMIV::MivBitstream::mode = MivDecoderMode::TMC2;

  const auto bitstreamPath = GENERATE(testBitstreams[0]);
  std::cout << "\n\nTEST_CASE Demultiplex: bitstreamPath=" << bitstreamPath.string() << '\n';
  std::ifstream stream{bitstreamPath, std::ios::binary};
  // TODO(BK): Need a bitstream that implements M53122
  // demultiplex(stream);
}

auto geoFrameServer(uint8_t atlasId, uint32_t frameId, Common::Vec2i frameSize) -> Depth10Frame {
  std::cout << "geoFrameServer: atlasId=" << int{atlasId} << ", frameId=" << frameId
            << ", frameSize=" << frameSize << '\n';
  return Depth10Frame{frameSize.x(), frameSize.y()};
}

auto occFrameServer(uint8_t atlasId, uint32_t frameId, Common::Vec2i frameSize)
    -> Occupancy10Frame {
  std::cout << "occFrameServer: atlasId=" << int{atlasId} << ", frameId=" << frameId
            << ", frameSize=" << frameSize << '\n';
  return Occupancy10Frame{frameSize.x(), frameSize.y()};
}

auto attrFrameServer(uint8_t atlasId, uint32_t frameId, Common::Vec2i frameSize)
    -> Texture444Frame {
  std::cout << "attrFrameServer: atlasId=" << int{atlasId} << ", frameId=" << frameId
            << ", frameSize=" << frameSize << '\n';
  return Texture444Frame{frameSize.x(), frameSize.y()};
}

TEST_CASE("Decode", "[V3C bitstream]") {
  TMIV::MivBitstream::mode = MivDecoderMode::TMC2;

  const auto bitstreamPath = GENERATE(testBitstreams[0]);
  std::cout << "\n\nTEST_CASE Decode: bitstreamPath=" << bitstreamPath.string() << '\n';
  std::ifstream stream{bitstreamPath, std::ios::binary};
  auto decoder = MivDecoder{stream};
  decoder.setOccFrameServer(occFrameServer);
  decoder.setGeoFrameServer(geoFrameServer);
  decoder.setAttrFrameServer(attrFrameServer);
  // TODO(BK): Need a bitstream that implements M53122
  // decoder.decode();
}
