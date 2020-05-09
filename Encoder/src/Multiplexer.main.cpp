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
#include <TMIV/MivBitstream/MivDecoderMode.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <cassert>
#include <filesystem>
#include <iostream>
#include <stdexcept>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::IO;
using namespace TMIV::MivBitstream;

using std::filesystem::path;

namespace TMIV::MivBitstream {
const MivDecoderMode mode = MivDecoderMode::MIV;
}

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
    openOutputBitstream();

    // Decode VPS & copy to output bitstream
    processIntermediateBitstream();

    // Append all video sub bitstreams.
    for (uint8_t j = 0; j <= m_vps->vps_atlas_count_minus1(); ++j) {
      checkRestrictions(j);
      if (m_vps->vps_geometry_video_present_flag(j)) {
        appendGvd(m_vps->vps_atlas_id(j));
      }
      if (m_vps->vps_occupancy_video_present_flag(j)) {
        appendOvd(m_vps->vps_atlas_id(j));
      }
      if (m_vps->vps_attribute_video_present_flag(j)) {
        const auto &ai = m_vps->attribute_information(j);
        for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
          const auto type = ai.ai_attribute_type_id(i);
          appendAvd(m_vps->vps_atlas_id(j), i, type);
        }
      }
    }
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

  void openOutputBitstream() {
    m_outStream.open(m_outputBitstreamPath, ios::binary);
    if (!m_outStream.good()) {
      throw runtime_error(
          format("Failed to open intermediate bitstream ({}) for reading.", m_outputBitstreamPath));
    }
  }

  void processIntermediateBitstream() {
    ifstream inStream{m_intermediateBitstreamPath, ios::binary};
    if (!inStream.good()) {
      throw runtime_error(format("Failed to open intermediate bitstream ({}) for reading.",
                                 m_intermediateBitstreamPath));
    }

    // This version of this multiplexer just puts the video sub bitstreams to the end. There is no
    // interleaving of V3C units. Copy the intermediate bitstream to the output bitstream.
    m_outStream << inStream.rdbuf();

    // Decode the VPS to learn which video sub bitstreams have to be appended.
    inStream.seekg(0);
    m_vps = decodeVps(inStream);
    cout << *m_vps;
    cout << "Appended " << m_intermediateBitstreamPath << '\n';
  }

  auto decodeVps(istream &stream) -> V3cParameterSet {
    m_ssvh = SampleStreamV3cHeader::decodeFrom(stream);
    const auto ssvu = SampleStreamV3cUnit::decodeFrom(stream, *m_ssvh);
    istringstream substream{ssvu.ssvu_v3c_unit()};
    const auto vpses = vector<V3cParameterSet>{};
    const auto vuh = V3cUnitHeader::decodeFrom(substream, vpses);
    if (vuh.vuh_unit_type() != VuhUnitType::V3C_VPS) {
      throw runtime_error("the first V3C unit has to be the VPS");
    }
    return V3cParameterSet::decodeFrom(substream);
  }

  void checkRestrictions(uint8_t atlasIdx) const {
    if (m_vps->vps_map_count_minus1(atlasIdx) > 0) {
      throw runtime_error("Having multiple maps is not supported.");
    }
    if (m_vps->vps_auxiliary_video_present_flag(atlasIdx)) {
      throw runtime_error("Auxiliary video is not supported.");
    }
  }

  void appendGvd(uint8_t atlasId) {
    auto vuh = V3cUnitHeader{VuhUnitType::V3C_GVD};
    vuh.vuh_v3c_parameter_set_id(m_vps->vps_v3c_parameter_set_id());
    vuh.vuh_atlas_id(atlasId);
    appendSubBitstream(vuh, format(m_gvdSubBitstreamPathFmt, int(atlasId)));
  }

  void appendOvd(uint8_t atlasId) {
    cout << "WARNING: OVD support is untested\n";
    auto vuh = V3cUnitHeader{VuhUnitType::V3C_OVD};
    vuh.vuh_v3c_parameter_set_id(m_vps->vps_v3c_parameter_set_id());
    vuh.vuh_atlas_id(atlasId);
    appendSubBitstream(vuh, format(*m_ovdSubBitstreamPathFmt, int(atlasId)));
  }

  void appendAvd(int atlasId, uint8_t attributeIdx, AiAttributeTypeId typeId) {
    auto vuh = V3cUnitHeader{VuhUnitType::V3C_AVD};
    vuh.vuh_v3c_parameter_set_id(m_vps->vps_v3c_parameter_set_id());
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
    const auto vpses = vector<V3cParameterSet>{*m_vps};
    vuh.encodeTo(substream, vpses);
    substream << inStream.rdbuf();

    auto ssvu = SampleStreamV3cUnit{substream.str()};
    ssvu.encodeTo(m_outStream, *m_ssvh);
    cout << "Appended " << subBitstreamPath << '\n';
  }

  path m_intermediateBitstreamPath;
  path m_outputBitstreamPath;
  string m_gvdSubBitstreamPathFmt;
  optional<string> m_avdSubBitstreamPathFmt;
  optional<string> m_ovdSubBitstreamPathFmt;

  ofstream m_outStream;
  optional<SampleStreamV3cHeader> m_ssvh;
  optional<V3cParameterSet> m_vps;
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
