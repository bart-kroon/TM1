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

#include <TMIV/Common/Application.h>
#include <TMIV/IO/IO.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <cassert>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <stdexcept>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::IO;
using namespace TMIV::MivBitstream;

using std::filesystem::path;

namespace TMIV::Encoder {
class Multiplexer : public Common::Application {
public:
  explicit Multiplexer(vector<const char *> argv)
      : Common::Application{"Multiplexer", move(argv)}
      , m_intermediateBitstreamPath{json().require("IntermediateBitstreamPath").asString()}
      , m_outputBitstreamPath{json().require("OutputBitstreamPath").asString()}
      , m_gvdSubBitstreamPathFmt{
            json().require("GeometryVideoDataSubBitstreamPathFmt").asString()} {
    if (auto node = json().optional("AttributeVideoDataSubBitstreamPathFmt"); node) {
      m_avdSubBitstreamPathFmt = node.asString();
    }
    if (auto node = json().optional("OccupancyVideoDataSubBitstreamPathFmt"); node) {
      m_ovdSubBitstreamPathFmt = node.asString();
    }
    checkParameters();
  }

  void run() override {
    // Decode V3C units and VPS
    readIntermediateBitstream();

    // Append all video sub bitstreams
    for (uint8_t j = 0; j <= m_vps.vps_atlas_count_minus1(); ++j) {
      checkRestrictions(j);

      if (m_vps.vps_geometry_video_present_flag(j)) {
        appendGvd(m_vps.vps_atlas_id(j));
      }
      if (m_vps.vps_occupancy_video_present_flag(j)) {
        appendOvd(m_vps.vps_atlas_id(j));
      }
      if (m_vps.vps_attribute_video_present_flag(j)) {
        const auto &ai = m_vps.attribute_information(j);
        for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
          const auto type = ai.ai_attribute_type_id(i);
          appendAvd(m_vps.vps_atlas_id(j), i, type);
        }
      }
    }

    writeOutputBitstream();
  }

private:
  void checkParameters() {
    if (!exists(m_intermediateBitstreamPath)) {
      throw runtime_error(format("The intermediate bitstream file ({}) does not exist.",
                                 m_intermediateBitstreamPath));
    }
    // We are daring enough to overwrite an existing bitstream, but we refuse to overwrite the
    // input bitstream. (Because we know we are stupid enough to do that at least once.)
    if (exists(m_outputBitstreamPath) &&
        equivalent(m_intermediateBitstreamPath, m_outputBitstreamPath)) {
      throw runtime_error(format("The intermediate bitstream file ({}) and the output bitstream "
                                 "file ({}) cannot be the same file.",
                                 m_intermediateBitstreamPath, m_outputBitstreamPath));
    }
  }

  void readIntermediateBitstream() {
    ifstream stream{m_intermediateBitstreamPath, ios::binary};
    if (!stream.good()) {
      throw runtime_error(format("Failed to open intermediate bitstream ({}) for reading.",
                                 m_intermediateBitstreamPath));
    }

    // Decode SSVH
    const auto ssvh = SampleStreamV3cHeader::decodeFrom(stream);

    // Decode first V3C unit, which has to contain the VPS
    const auto ssvu0 = SampleStreamV3cUnit::decodeFrom(stream, ssvh);

    // Decode the VPS
    istringstream substream{ssvu0.ssvu_v3c_unit()};
    const auto vuh = V3cUnitHeader::decodeFrom(substream);
    if (vuh.vuh_unit_type() != VuhUnitType::V3C_VPS) {
      throw runtime_error("the first V3C unit has to be the VPS");
    }
    m_vps = V3cParameterSet::decodeFrom(substream);
    cout << m_vps;

    // Append the first V3C unit
    m_units.push_back(ssvu0.ssvu_v3c_unit());

    // Append the remaining V3C units
    while (!stream.eof()) {
      const auto ssvu = SampleStreamV3cUnit::decodeFrom(stream, ssvh);
      m_units.push_back(ssvu.ssvu_v3c_unit());
      stream.peek();
    }

    cout << "Appended " << m_intermediateBitstreamPath << " with a total of " << m_units.size()
         << " V3C units including the VPS\n";
  }

  void checkRestrictions(uint8_t atlasIdx) const {
    if (m_vps.vps_map_count_minus1(atlasIdx) > 0) {
      throw runtime_error("Having multiple maps is not supported.");
    }
    if (m_vps.vps_auxiliary_video_present_flag(atlasIdx)) {
      throw runtime_error("Auxiliary video is not supported.");
    }
  }

  void appendGvd(uint8_t atlasId) {
    auto vuh = V3cUnitHeader{VuhUnitType::V3C_GVD};
    vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id());
    vuh.vuh_atlas_id(atlasId);
    appendSubBitstream(vuh, format(m_gvdSubBitstreamPathFmt, int(atlasId)));
  }

  void appendOvd(uint8_t atlasId) {
    //cout << "WARNING: OVD support is untested\n"; Now this has been tested successfuly :)
    auto vuh = V3cUnitHeader{VuhUnitType::V3C_OVD};
    vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id());
    vuh.vuh_atlas_id(atlasId);
    appendSubBitstream(vuh, format(*m_ovdSubBitstreamPathFmt, int(atlasId)));
  }

  void appendAvd(int atlasId, uint8_t attributeIdx, AiAttributeTypeId typeId) {
    auto vuh = V3cUnitHeader{VuhUnitType::V3C_AVD};
    vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id());
    vuh.vuh_atlas_id(atlasId);
    vuh.vuh_attribute_index(attributeIdx);
    appendSubBitstream(vuh, format(*m_avdSubBitstreamPathFmt, codeOf(typeId), int(atlasId)));
  }

  void appendSubBitstream(const V3cUnitHeader &vuh, const path &subBitstreamPath) {
    ifstream inStream{subBitstreamPath, ios::binary};
    if (!inStream.good()) {
      throw runtime_error(
          format("Failed to open sub bitstream ({}) for reading", subBitstreamPath));
    }
    ostringstream substream;
    vuh.encodeTo(substream);
    substream << inStream.rdbuf();

    cout << "Appended " << subBitstreamPath << '\n';
    m_units.push_back(substream.str());
  }

  void writeOutputBitstream() const {
    // Find size of largest unit
    const auto maxSize =
        max_element(cbegin(m_units), cend(m_units), [](const string &a, const string &b) {
          return a.size() < b.size();
        })->size();

    // Calculate how many bytes are needed to store that size
    auto precisionBytesMinus1 = uint8_t{};
    while (maxSize >= uint64_t(1) << 8 * (precisionBytesMinus1 + 1)) {
      ++precisionBytesMinus1;
    }

    // Write the sample stream header
    ofstream stream{m_outputBitstreamPath, ios::binary};
    const auto ssvh = SampleStreamV3cHeader{precisionBytesMinus1};
    ssvh.encodeTo(stream);
    cout << '\n' << ssvh;

    // Write the units
    for (const auto &unit : m_units) {
      const auto ssvu = SampleStreamV3cUnit{unit};
      ssvu.encodeTo(stream, ssvh);
      cout << '\n' << ssvu;

      // Print the V3C unit header (for fun, why not)
      istringstream stream{unit};
      const auto vuh = V3cUnitHeader::decodeFrom(stream);
      cout << vuh;
    }
  }

  path m_intermediateBitstreamPath;
  path m_outputBitstreamPath;
  string m_gvdSubBitstreamPathFmt;
  optional<string> m_avdSubBitstreamPathFmt;
  optional<string> m_ovdSubBitstreamPathFmt;

  V3cParameterSet m_vps;
  vector<string> m_units;
};
} // namespace TMIV::Encoder

auto main(int argc, char *argv[]) -> int {
  try {
    TMIV::Encoder::Multiplexer app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (runtime_error &e) {
    cerr << e.what() << endl;
    return 1;
  }
}
