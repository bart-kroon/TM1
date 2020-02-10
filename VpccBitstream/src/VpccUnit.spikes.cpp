/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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
#include <TMIV/VpccBitstream/VpccSampleStreamFormat.h>
#include <TMIV/VpccBitstream/VpccUnit.h>

#include <fstream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::VpccBitstream;

const auto bitstreamPath = "C://Data//longdress_vox10_GOF0.vpcc";

// Copied from PCCBitstream::readHeader (not in the specification)
const uint32_t PCCTMC2ContainerMagicNumber = 23021981;
const uint32_t PCCTMC2ContainerVersion = 1;

auto dumpVpccUnitPayload(streampos position, const SampleStreamVpccUnit &ssvu,
                         const VpccUnitHeader &vuh) {
  ostringstream path;
  path << "vpcc_unit_" << position << '_' << vuh.vuh_unit_type();
  if (vuh.vuh_unit_type() != VuhUnitType::VPCC_VPS) {
    path << '_' << int(vuh.vuh_vpcc_parameter_set_id()) << '_' << int(vuh.vuh_atlas_id());
  }
  if (vuh.vuh_unit_type() == VuhUnitType::VPCC_AVD) {
    path << '_' << int(vuh.vuh_attribute_index()) << '_'
         << int(vuh.vuh_attribute_dimension_index());
  }
  if (vuh.vuh_unit_type() == VuhUnitType::VPCC_AVD ||
      vuh.vuh_unit_type() == VuhUnitType::VPCC_GVD) {
    path << '_' << int(vuh.vuh_map_index()) << '_' << boolalpha << vuh.vuh_raw_video_flag();
  }
  if (vuh.vuh_unit_type() == VuhUnitType::VPCC_VPS || vuh.vuh_unit_type() == VuhUnitType::VPCC_AD) {
    path << ".bit";
  } else {
    path << ".mp4";
  }

  ofstream file{path.str(), ios::binary};
  auto payload = ssvu.ssvu_vpcc_unit();
  payload.erase(payload.begin(), payload.begin() + 4);
  file.write(payload.data(), payload.size());
}

TEST_CASE("Parse V-PCC sample stream", "[VPCC Unit]") {
  ifstream stream{bitstreamPath, ios::binary};
  REQUIRE(stream.good());

  stream.seekg(0, ios::end);
  const auto filesize = stream.tellg();
  cout << "[ 0 ]: File size is " << filesize << " bytes\n";
  stream.seekg(0);

  REQUIRE(getUint32(stream) == PCCTMC2ContainerMagicNumber);
  REQUIRE(getUint32(stream) == PCCTMC2ContainerVersion);
  const auto totalSize = getUint64(stream);
  cout << "totalSize=" << totalSize << " (reported)\n";

  cout << "[ " << stream.tellg() << " ]: ";
  const auto ssvh = SampleStreamVpccHeader::decodeFrom(stream);
  cout << ssvh;

  auto vpses = vector<VpccParameterSet>{};

  while (stream.tellg() != filesize) {
    const auto position = stream.tellg();
    cout << "[ " << position << " ]: ";
    const auto ssvu = SampleStreamVpccUnit::decodeFrom(stream, ssvh);
    cout << ssvu;

    istringstream substream{ssvu.ssvu_vpcc_unit()};
    const auto vuh = VpccUnitHeader::decodeFrom(substream, vpses);
    cout << vuh;

    dumpVpccUnitPayload(position, ssvu, vuh);

    const auto vp = VpccPayload::decodeFrom(substream, vuh);
    cout << vp;

    if (auto vps = get_if<VpccParameterSet>(&vp.payload()); vps != nullptr) {
      while (vps->vps_vpcc_parameter_set_id() >= vpses.size()) {
        vpses.emplace_back();
      }
      cout << "vpses[" << int(vps->vps_vpcc_parameter_set_id()) << "] := vps\n";
      vpses[vps->vps_vpcc_parameter_set_id()] = *vps;
    }
  }

  cout << "[ " << stream.tellg() << " ].\n";
}
