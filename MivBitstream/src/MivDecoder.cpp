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

#include <TMIV/MivBitstream/MivDecoder.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Bytestream.h>
#include <TMIV/MivBitstream/VpccSampleStreamFormat.h>
#include <TMIV/MivBitstream/VpccUnit.h>

#include <istream>
#include <sstream>
#include <variant>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
MivDecoder::MivDecoder(std::istream &stream)
    : m_stream{stream}, m_ssvh{sampleStreamVpccHeader(stream)} {}

auto MivDecoder::decodeVpccUnit() -> bool {
  VERIFY_VPCCBITSTREAM(m_stream.good());
  const auto ssvu = SampleStreamVpccUnit::decodeFrom(m_stream, m_ssvh);
  VERIFY_VPCCBITSTREAM(m_stream.good());

  istringstream substream{ssvu.ssvu_vpcc_unit()};
  const auto vu = VpccUnit::decodeFrom(substream, m_vps, ssvu.ssvu_vpcc_unit_size());
  visit([this, &vu](const auto &payload) { onVpccPayload(vu.vpcc_unit_header(), payload); },
        vu.vpcc_payload().payload());

  m_stream.peek();
  return !m_stream.eof();
}

void MivDecoder::decode() {
  while (decodeVpccUnit()) {
    ;
  }
}

void MivDecoder::onVpccPayload(const VpccUnitHeader & /* vuh */,
                               const std::monostate & /* payload */) {
  VPCCBITSTREAM_ERROR("V-PCC payload of unknown type");
}

void MivDecoder::onVpccPayload(const VpccUnitHeader & /*vuh*/, const VpccParameterSet &vps) {
  const auto id = vps.vps_vpcc_parameter_set_id();
  while (m_vps.size() <= id) {
    m_vps.emplace_back();
    m_sequence.emplace_back();
  }
  m_vps[id] = vps;
  m_sequence[id] = Sequence{};
  m_sequence[id].atlas.resize(vps.vps_atlas_count());
}

void MivDecoder::onVpccPayload(const VpccUnitHeader &vuh, const AtlasSubBitstream &ad) {
  for (const auto &nu : ad.nal_units()) {
    onNalUnit(vuh, nu);
  }
}

void MivDecoder::onVpccPayload(const VpccUnitHeader & /*vuh*/, const VideoSubBitstream & /*vd*/) {
  // TODO(BK): Implement
}

void MivDecoder::onNalUnit(const VpccUnitHeader &vuh, const NalUnit &nu) {
  switch (nu.nal_unit_header().nal_unit_type()) {
  case NalUnitType::NAL_TRAIL:
  case NalUnitType::NAL_TSA:
  case NalUnitType::NAL_STSA:
  case NalUnitType::NAL_RADL:
  case NalUnitType::NAL_RASL:
  case NalUnitType::NAL_SKIP:
  case NalUnitType::NAL_BLA_W_LP:
  case NalUnitType::NAL_BLA_W_RADL:
  case NalUnitType::NAL_BLA_N_LP:
  case NalUnitType::NAL_GBLA_W_LP:
  case NalUnitType::NAL_GBLA_W_RADL:
  case NalUnitType::NAL_GBLA_N_LP:
  case NalUnitType::NAL_IDR_W_RADL:
  case NalUnitType::NAL_IDR_N_LP:
  case NalUnitType::NAL_GIDR_W_RADL:
  case NalUnitType::NAL_GIDR_N_LP:
  case NalUnitType::NAL_CRA:
  case NalUnitType::NAL_GCRA:
    return decodeAcl(vuh, nu);
  case NalUnitType::NAL_ASPS:
    return decodeAsps(vuh, nu);
  case NalUnitType::NAL_AFPS:
    return decodeAfps(vuh, nu);
  case NalUnitType::NAL_AUD:
    return decodeAud(vuh, nu);
  case NalUnitType::NAL_VPCC_AUD:
    return decodeVpccAud(vuh, nu);
  case NalUnitType::NAL_EOS:
    return decodeEos(vuh, nu);
  case NalUnitType::NAL_EOB:
    return decodeEob(vuh, nu);
  case NalUnitType::NAL_FD:
    return decodeFd(vuh, nu);
  case NalUnitType::NAL_PREFIX_NSEI:
    return decodePrefixNSei(vuh, nu);
  case NalUnitType::NAL_SUFFIX_NSEI:
    return decodeSuffixNSei(vuh, nu);
  case NalUnitType::NAL_PREFIX_ESEI:
    return decodePrefixESei(vuh, nu);
  case NalUnitType::NAL_SUFFIX_ESEI:
    return decodeSuffixESei(vuh, nu);
  default:
    return onUnknownNalUnit(vuh, nu);
  }
}

void MivDecoder::onUnknownNalUnit(const VpccUnitHeader & /*vuh*/, const NalUnit &nu) {
  cout << "WARNING: Ignoring NAL unit of unknown type " << nu.nal_unit_header().nal_unit_type()
       << '\n';
}

void MivDecoder::onAtgl(const VpccUnitHeader & /* vuh */, const NalUnitHeader & /* nuh */,
                        AtlasTileGroupLayerRBSP /* atgl */) {
  // TODO(BK): Implement
}

void MivDecoder::onAsps(const VpccUnitHeader &vuh, const NalUnitHeader & /* nuh */,
                        AtlasSequenceParameterSetRBSP asps) {
  auto &atlas_ = atlas(vuh);
  while (atlas_.asps.size() <= asps.asps_atlas_sequence_parameter_set_id()) {
    atlas_.asps.emplace_back();
  }
  atlas_.asps[asps.asps_atlas_sequence_parameter_set_id()] = move(asps);
}

void MivDecoder::onAfps(const VpccUnitHeader &vuh, const NalUnitHeader & /*nuh*/,
                        AtlasFrameParameterSetRBSP afps) {
  auto &atlas_ = atlas(vuh);
  while (atlas_.afps.size() <= afps.afps_atlas_frame_parameter_set_id()) {
    atlas_.afps.emplace_back();
  }
  atlas_.afps[afps.afps_atlas_frame_parameter_set_id()] = afps;
}

void MivDecoder::onAud(const VpccUnitHeader & /* vuh */, const NalUnitHeader & /* nuh */,
                       AccessUnitDelimiterRBSP /* aud */) {
  // TODO(BK): Implement
}

void MivDecoder::onVpccAud(const VpccUnitHeader & /* vuh */, const NalUnitHeader & /* nuh */,
                           AccessUnitDelimiterRBSP /* aud */) {
  // TODO(BK): Implement
}

void MivDecoder::onEos(const VpccUnitHeader & /* vuh */, const NalUnitHeader & /* nuh */,
                       EndOfSequenceRBSP /* eos */) {
  // TODO(BK): Implement
}

void MivDecoder::onEob(const VpccUnitHeader & /* vuh */, const NalUnitHeader & /* nuh */,
                       EndOfAtlasSubBitstreamRBSP /* eob */) {
  // TODO(BK): Implement
}

void MivDecoder::onPrefixNSei(const VpccUnitHeader & /* vuh */, const NalUnitHeader & /* nuh */,
                              SeiRBSP /* sei */) {
  // TODO(BK): Implement
}

void MivDecoder::onSuffixNSei(const VpccUnitHeader & /* vuh */, const NalUnitHeader & /* nuh */,
                              SeiRBSP /* sei */) {
  // TODO(BK): Implement
}

void MivDecoder::onPrefixESei(const VpccUnitHeader & /* vuh */, const NalUnitHeader & /* nuh */,
                              SeiRBSP /* sei */) {
  // TODO(BK): Implement
}

void MivDecoder::onSuffixESei(const VpccUnitHeader & /* vuh */, const NalUnitHeader & /* nuh */,
                              SeiRBSP /* sei */) {
  // TODO(BK): Implement
}

auto MivDecoder::sampleStreamVpccHeader(istream &stream) -> SampleStreamVpccHeader {
  if (mode == MivDecoder::Mode::TMC2) {
    // Skip TMC2 header
    const uint32_t PCCTMC2ContainerMagicNumber = 23021981;
    const uint32_t PCCTMC2ContainerVersion = 1;
    VERIFY_TMC2BITSTREAM(getUint32(stream) == PCCTMC2ContainerMagicNumber);
    VERIFY_TMC2BITSTREAM(getUint32(stream) == PCCTMC2ContainerVersion);
    VERIFY_TMC2BITSTREAM(getUint64(stream) == 0);
  }
  return SampleStreamVpccHeader::decodeFrom(stream);
}

void MivDecoder::decodeAsps(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  onAsps(vuh, nu.nal_unit_header(), AtlasSequenceParameterSetRBSP::decodeFrom(stream));
}

void MivDecoder::decodeAfps(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  onAfps(vuh, nu.nal_unit_header(), AtlasFrameParameterSetRBSP::decodeFrom(stream, asps(vuh)));
}

void MivDecoder::decodeAud(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  onAud(vuh, nu.nal_unit_header(), AccessUnitDelimiterRBSP::decodeFrom(stream));
}

void MivDecoder::decodeVpccAud(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  onVpccAud(vuh, nu.nal_unit_header(), AccessUnitDelimiterRBSP::decodeFrom(stream));
}

void MivDecoder::decodeEos(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  onEos(vuh, nu.nal_unit_header(), EndOfSequenceRBSP::decodeFrom(stream));
}

void MivDecoder::decodeEob(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  onEob(vuh, nu.nal_unit_header(), EndOfAtlasSubBitstreamRBSP::decodeFrom(stream));
}

void MivDecoder::decodeFd(const VpccUnitHeader & /* vuh */, const NalUnit & /* nu */) {
  cout << "Ignoring filler data\n";
}

void MivDecoder::decodePrefixNSei(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  onPrefixNSei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::decodeSuffixNSei(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  onSuffixNSei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::decodePrefixESei(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  onPrefixESei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::decodeSuffixESei(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  onSuffixESei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

auto MivDecoder::sequence(const VpccUnitHeader &vuh) const -> const Sequence & {
  VERIFY_VPCCBITSTREAM(vuh.vuh_vpcc_parameter_set_id() < m_vps.size());
  return m_sequence[vuh.vuh_vpcc_parameter_set_id()];
}

auto MivDecoder::sequence(const VpccUnitHeader &vuh) -> Sequence & {
  VERIFY_VPCCBITSTREAM(vuh.vuh_vpcc_parameter_set_id() < m_vps.size());
  return m_sequence[vuh.vuh_vpcc_parameter_set_id()];
}

auto MivDecoder::atlas(const VpccUnitHeader &vuh) const -> const Atlas & {
  const auto &sequence_ = sequence(vuh);

  if (vuh.vuh_atlas_id() == specialAtlasId) {
    return specialAtlas(vuh);
  }

  VERIFY_VPCCBITSTREAM(vuh.vuh_atlas_id() < sequence_.atlas.size());
  return sequence_.atlas[vuh.vuh_atlas_id()];
}

auto MivDecoder::atlas(const VpccUnitHeader &vuh) -> Atlas & {
  auto &sequence_ = sequence(vuh);

  if (vuh.vuh_atlas_id() == specialAtlasId) {
    return specialAtlas(vuh);
  }

  VERIFY_VPCCBITSTREAM(vuh.vuh_atlas_id() < sequence_.atlas.size());
  return sequence_.atlas[vuh.vuh_atlas_id()];
}

auto MivDecoder::specialAtlas(const VpccUnitHeader &vuh) const -> const Atlas & {
  const auto &sequence_ = sequence(vuh);

  VERIFY_MIVBITSTREAM(sequence_.specialAtlas.has_value());
  return *sequence_.specialAtlas;
}

auto MivDecoder::specialAtlas(const VpccUnitHeader &vuh) -> Atlas & {
  auto &sequence_ = sequence(vuh);

  VERIFY_MIVBITSTREAM(sequence_.specialAtlas.has_value());
  return *sequence_.specialAtlas;
}

auto MivDecoder::asps(const VpccUnitHeader &vuh) const
    -> const std::vector<AtlasSequenceParameterSetRBSP> & {
  auto &atlas_ = atlas(vuh);
  if (atlas_.asps.empty()) {
    return specialAtlas(vuh).asps;
  }
  return atlas_.asps;
}

auto MivDecoder::afps(const VpccUnitHeader &vuh) const
    -> const std::vector<AtlasFrameParameterSetRBSP> & {
  auto &atlas_ = atlas(vuh);
  if (atlas_.afps.empty()) {
    return specialAtlas(vuh).afps;
  }
  return atlas_.afps;
}
} // namespace TMIV::MivBitstream
