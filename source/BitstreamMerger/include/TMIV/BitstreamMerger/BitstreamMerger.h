/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#ifndef TMIV_BITSTREAMMERGER_BITSTREAMMERGER_H
#define TMIV_BITSTREAMMERGER_BITSTREAMMERGER_H

#include <TMIV/Common/Json.h>
#include <TMIV/Common/Source.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <fstream>
#include <string>
#include <vector>

namespace TMIV::BitstreamMerger {
class BitstreamMerger {
public:
  void readInputBitstream(std::istream &stream);
  void readOutofBandMetadata(const Common::Json &node);
  void updatePackingInformation();
  void writeOutputBitstream(std::ostream &stream);
  void writeOutOfBandMetadata(std::ostream &stream);
  static void reportSummary(std::streampos bytesWritten);

  // ISO/IEC 23090-12 Annex B.4.2 output
  struct UnpackedVideoComponentResolutions {
    int32_t unpckOccWidth{};
    int32_t unpckOccHeight{};
    int32_t unpckGeoWidth{};
    int32_t unpckGeoHeight{};
    std::vector<int32_t> unpckAttrWidth;
    std::vector<int32_t> unpckAttrHeight;
  };

private:
  MivBitstream::V3cParameterSet m_vps{};
  std::vector<std::string> m_units{};
  static constexpr uint8_t m_vpsId = 0;
  uint32_t m_bitDepth{};
  std::vector<Common::Json> m_irapFrameIndices;
  Common::Vec2i m_packedVideoResolution{0, 0};

  static void updateOccupancyInformation(MivBitstream::PackingInformation &packinginformation,
                                         const MivBitstream::PackingInformation &pin);
  static void updateGeometryInformation(MivBitstream::PackingInformation &packinginformation,
                                        const MivBitstream::PackingInformation &pin);
  static void updateAttributeInformation(MivBitstream::PackingInformation &packinginformation,
                                         const MivBitstream::PackingInformation &pin,
                                         const MivBitstream::V3cParameterSet &vps,
                                         MivBitstream::AtlasId j);
  static void setAttributePinRegion(MivBitstream::PackingInformation &packingInformation,
                                    const MivBitstream::V3cParameterSet &vps,
                                    MivBitstream::AtlasId j);
  void calculatePackedVIdeoComponentResolution(MivBitstream::AtlasId atlasId);
};

} // namespace TMIV::BitstreamMerger
#endif // TMIV_ENCODER_BITSTREAMMERGER_H
