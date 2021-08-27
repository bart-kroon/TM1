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

#ifndef TMIV_ENCODER_MULTIPLEXER_H
#define TMIV_ENCODER_MULTIPLEXER_H

#include <TMIV/Common/Json.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <string>
#include <vector>

namespace TMIV::Encoder {
class Multiplexer {
public:
  using AttributeVideoBitstreamServer = std::function<std::unique_ptr<std::istream>(
      MivBitstream::AiAttributeTypeId, const MivBitstream::AtlasId &, int32_t)>;
  using GeometryVideoBitstreamServer =
      std::function<std::unique_ptr<std::istream>(const MivBitstream::AtlasId &)>;
  using OccupancyVideoBitstreamServer = GeometryVideoBitstreamServer;
  using PackedVideoBitstreamServer = GeometryVideoBitstreamServer;

  explicit Multiplexer(Common::Json packingInformationNode);

  void setAttributeVideoBitstreamServer(AttributeVideoBitstreamServer server);
  void setGeometryVideoBitstreamServer(GeometryVideoBitstreamServer server);
  void setOccupancyVideoBitstreamServer(OccupancyVideoBitstreamServer server);
  void setPackedVideoBitstreamServer(PackedVideoBitstreamServer server);

  void readInputBitstream(std::istream &stream);
  void appendVideoSubBitstreams();
  void writeOutputBitstream(std::ostream &stream) const;
  void addPackingInformation();

  [[nodiscard]] auto numberOfV3cUnits() const { return m_units.size(); }

private:
  void checkRestrictions(MivBitstream::AtlasId atlasId) const;
  void appendGvd(MivBitstream::AtlasId atlasId);
  void appendOvd(MivBitstream::AtlasId atlasId);
  void appendAvd(MivBitstream::AtlasId atlasId, uint8_t attributeIdx,
                 MivBitstream::AiAttributeTypeId typeId);
  void appendPvd(MivBitstream::AtlasId atlasId);
  void appendVideoSubBitstream(const MivBitstream::V3cUnitHeader &vuh,
                               std::unique_ptr<std::istream> stream);

  MivBitstream::V3cParameterSet m_vps{};
  std::vector<std::string> m_units{};
  Common::Json m_packingInformationNode{};

  AttributeVideoBitstreamServer m_openAttributeVideoBitstream{};
  GeometryVideoBitstreamServer m_openGeometryVideoBitstream{};
  OccupancyVideoBitstreamServer m_openOccupancyVideoBitstream{};
  PackedVideoBitstreamServer m_openPackedVideoBitstream{};
  void updateOccupancyInformation(MivBitstream::PackingInformation &packingInformation,
                                  const MivBitstream::AtlasId &atlasId);
  void updateGeometryInformation(MivBitstream::PackingInformation &packingInformation,
                                 const MivBitstream::AtlasId &atlasId);
  void updateAttributeInformation(MivBitstream::PackingInformation &packingInformation,
                                  const MivBitstream::AtlasId &atlasId);
};
} // namespace TMIV::Encoder
#endif // TMIV_ENCODER_MULTIPLEXER_H
