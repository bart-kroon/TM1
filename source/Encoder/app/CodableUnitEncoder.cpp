/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include "CodableUnitEncoder.h"

#include <TMIV/Encoder/EncodeMiv.h>
#include <TMIV/Encoder/V3cSampleSink.h>

namespace TMIV::Encoder {
CodableUnitEncoder::CodableUnitEncoder(const Common::Json &config, IO::Placeholders placeholders)
    : m_config{config}
    , m_placeholders{std::move(placeholders)}
    , m_outputBitstreamPath{IO::outputBitstreamPath(config, m_placeholders)}
    , m_outputBitstream{m_outputBitstreamPath, std::ios::binary}
    , m_mivEncoder{encodeMiv(v3cSampleSink(m_outputBitstream),
                             config.require("rewriteParameterSets").as<bool>())} {
  if (!m_outputBitstream.good()) {
    throw std::runtime_error(fmt::format("Failed to open {} for writing.", m_outputBitstreamPath));
  }
}

void CodableUnitEncoder::encode(CodableUnit frame) {
  Common::logInfo("Saving frame {}.", m_outputFrameIdx);

  if (frame.hasAcl) {
    m_mivEncoder(std::move(frame.encoderParams));
  }
  if (m_outputFrameIdx == 0) {
    IO::saveOutOfBandMetadata(m_config, m_placeholders, saveV3cFrameList(frame.v3cFrameList));
  } else {
    saveV3cFrameList(frame.v3cFrameList);
  }
  ++m_outputFrameIdx;
}

auto CodableUnitEncoder::saveV3cFrameList(const Common::V3cFrameList &v3cFrameList) const
    -> Common::Json::Array {
  auto metadata = Common::Json::Array{};

  for (size_t atlasIdx = 0; atlasIdx < v3cFrameList.size(); ++atlasIdx) {
    const auto atlasId = MivBitstream::AtlasId{atlasIdx};
    const auto sub = saveAtlasFrame(atlasId, m_outputFrameIdx, v3cFrameList[atlasIdx]);
    metadata.insert(metadata.end(), sub.cbegin(), sub.cend());
  }
  return metadata;
}

auto CodableUnitEncoder::saveAtlasFrame(MivBitstream::AtlasId atlasId, int32_t frameIdx,
                                        const Common::V3cFrame &frame) const
    -> Common::Json::Array {
  using VUH = MivBitstream::V3cUnitHeader;
  using VUT = MivBitstream::VuhUnitType;
  using ATI = MivBitstream::AiAttributeTypeId;

  auto metadata = Common::Json::Array{};
  uint8_t attrIdx{};

  const auto save = [this, frameIdx, &metadata, &attrIdx](const Common::Frame<> &component, VUH vuh,
                                                          ATI attrTypeId = ATI::ATTR_UNSPECIFIED) {
    if (!component.empty()) {
      metadata.emplace_back(IO::saveOutOfBandVideoFrame(m_config, m_placeholders, yuv420(component),
                                                        vuh, frameIdx, attrTypeId));
      if (vuh.vuh_unit_type() == VUT::V3C_AVD) {
        ++attrIdx;
      }
    }
  };

  static constexpr uint8_t m_vpsId = 0;
  save(frame.occupancy, VUH::ovd(m_vpsId, atlasId));
  save(frame.geometry, VUH::gvd(m_vpsId, atlasId));
  save(frame.texture, VUH::avd(m_vpsId, atlasId, attrIdx), ATI::ATTR_TEXTURE);
  save(frame.transparency, VUH::avd(m_vpsId, atlasId, attrIdx), ATI::ATTR_TRANSPARENCY);
  save(frame.packed, VUH::pvd(m_vpsId, atlasId));
  return metadata;
}

void CodableUnitEncoder::flush() { m_mivEncoder(std::nullopt); }
} // namespace TMIV::Encoder