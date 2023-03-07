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

#include <TMIV/Multiplexer/Multiplexer.h>

#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/Decoder.h>
#include <TMIV/Common/FlatMap.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/MivBitstream/Formatters.h>

namespace TMIV::Multiplexer {
using VU = MivBitstream::V3cUnit;
using VUT = MivBitstream::VuhUnitType;
using VUH = MivBitstream::V3cUnitHeader;
using NU = MivBitstream::NalUnit;
using NUT = MivBitstream::NalUnitType;

namespace {
class Multiplexer : public Common::Decoder<VU, VU> {
public:
  Multiplexer(Common::Source<VU> source, CodedVideoSequenceSourceFactory factory)
      : Common::Decoder<VU, VU>{std::move(source)}, m_factory{std::move(factory)} {}

protected:
  auto decodeSome() -> bool override {
    if (auto unit = pull()) {
      push(*unit);

      const auto vuh = unit->v3c_unit_header();
      Common::logInfo("Copy V3C unit: {}", vuh.summary());

      if (m_videoSources.empty()) {
        VERIFY_MIVBITSTREAM(vuh.vuh_unit_type() == VUT::V3C_VPS);
        createVideoSources(unit->v3c_unit_payload().v3c_parameter_set());
      }

      if (vuh.vuh_unit_type() == VUT::V3C_CAD) {
        const auto &nalUnits = unit->v3c_unit_payload().atlas_sub_bitstream().nal_units();
        const auto irapCount = std::count_if(nalUnits.cbegin(), nalUnits.cend(), [](const NU &nu) {
          return nu.nal_unit_header().nal_unit_type() == NUT::NAL_CAF_IDR;
        });
        pushCodedVideoSequences(irapCount);
      }
      return true;
    }
    return false;
  }

private:
  void createVideoSources(const MivBitstream::V3cParameterSet &vps) {
    for (const auto vuh : videoVuhs(vps)) {
      auto attrTypeId = MivBitstream::AiAttributeTypeId::ATTR_UNSPECIFIED;

      if (vuh.vuh_unit_type() == VUT::V3C_AVD) {
        const auto &ai = vps.attribute_information(vuh.vuh_atlas_id());
        attrTypeId = ai.ai_attribute_type_id(vuh.vuh_attribute_index());
      }

      Common::logInfo("Creating video source for: {}", vuh.summary());
      m_videoSources[vuh] = m_factory(vps, vuh, attrTypeId);
    }
    if (m_videoSources.empty()) {
      MIVBITSTREAM_ERROR(
          "The input bitstream is invalid. MIV requires at least one video sub-bitstream.");
    }
  }

  void pushCodedVideoSequences(size_t irapCount) {
    for (auto &[vuh, videoSource] : m_videoSources) {
      std::ostringstream vsb;

      for (size_t i = 0; i < irapCount; ++i) {
        if (auto units = videoSource()) {
          for (const auto &unit : *units) {
            const auto size = Common::downCast<int32_t>(unit.size());
            Common::logInfo("[{}] NAL unit: {} bytes", vuh.summary(), unit.size());
            Common::putUint32(vsb, size);
            vsb.write(unit.data(), size);
          }
        } else {
          MIVBITSTREAM_ERROR("Not enough IRAP AU's in the video sub-bitstream");
        }
      }

      push({vuh, MivBitstream::VideoSubBitstream{vsb.str()}});
    }
  }

  CodedVideoSequenceSourceFactory m_factory;
  Common::FlatMap<VUH, Common::Source<std::vector<std::string>>> m_videoSources;
};
} // namespace

auto multiplex(Common::Source<VU> source, CodedVideoSequenceSourceFactory factory)
    -> Common::Source<VU> {
  return [muxer = std::make_shared<Multiplexer>(std::move(source), std::move(factory))]() {
    return (*muxer)();
  };
}
} // namespace TMIV::Multiplexer
