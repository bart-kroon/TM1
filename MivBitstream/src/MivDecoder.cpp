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

#include <cassert>
#include <istream>
#include <sstream>
#include <variant>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
namespace {
auto sampleStreamVpccHeader(istream &stream) -> SampleStreamVpccHeader {
  if (MivDecoder::mode == MivDecoder::Mode::TMC2) {
    // Skip TMC2 header
    const uint32_t PCCTMC2ContainerMagicNumber = 23021981;
    const uint32_t PCCTMC2ContainerVersion = 1;
    VERIFY_TMC2BITSTREAM(getUint32(stream) == PCCTMC2ContainerMagicNumber);
    VERIFY_TMC2BITSTREAM(getUint32(stream) == PCCTMC2ContainerVersion);
    VERIFY_TMC2BITSTREAM(getUint64(stream) == 0);
  }
  return SampleStreamVpccHeader::decodeFrom(stream);
}
} // namespace

// Decoder interface ///////////////////////////////////////////////////////////////////////////////

MivDecoder::MivDecoder(istream &stream, GeoFrameServer geoFrameServer,
                       AttrFrameServer attrFrameServer)
    : m_stream{stream}, m_geoFrameServer{move(geoFrameServer)},
      m_attrFrameServer{move(attrFrameServer)}, m_ssvh{sampleStreamVpccHeader(stream)} {}

auto MivDecoder::decodeVpccUnit() -> bool {
  VERIFY_VPCCBITSTREAM(m_stream.good());
  const auto first = m_stream.tellg();
  const auto ssvu = SampleStreamVpccUnit::decodeFrom(m_stream, m_ssvh);
  VERIFY_VPCCBITSTREAM(m_stream.good());
  const auto last = m_stream.tellg();
  cout << "Sample stream V-PCC unit at [" << first << ", " << last << ") bytes:" << ssvu;

  istringstream substream{ssvu.ssvu_vpcc_unit()};
  const auto vu = VpccUnit::decodeFrom(substream, m_vpsV, ssvu.ssvu_vpcc_unit_size());
  cout << vu.vpcc_unit_header();
  visit([this, &vu](const auto &payload) { decodeVpccPayload(vu.vpcc_unit_header(), payload); },
        vu.vpcc_payload().payload());

  m_stream.peek();
  return !m_stream.eof();
}

void MivDecoder::decode() {
  while (!m_stop && decodeVpccUnit()) {
  }
}

// Decoder output //////////////////////////////////////////////////////////////////////////////////

void MivDecoder::outputSequence(const VpccParameterSet &vps) {
  for (const auto &x : onSequence) {
    if (!x(vps)) {
      m_stop = true;
    }
  }
}

void MivDecoder::outputFrame(const VpccUnitHeader &vuh) {
  assert(haveFrame(vuh));
  AccessUnit au;
  au.vps = &vps(vuh);

  auto &sequence_ = sequence(vuh);
  au.frameId = ++sequence_.frameId;
  au.atlas.resize(sequence_.atlas.size());

  for (uint8_t atlasId = 0; atlasId < au.atlas.size(); ++atlasId) {
    auto &atlas = sequence_.atlas[atlasId];
    assert(!atlas.atgl.empty());

    auto &aau = au.atlas[atlasId];
    aau.atgl = atlas.atgl.front();
    atlas.atgl.erase(atlas.atgl.begin());
    const auto &atgh = aau.atgl.atlas_tile_group_header();

    aau.afps = atlas.afpsV[atgh.atgh_atlas_frame_parameter_set_id()];

    aau.asps = atlas.aspsV[aau.afps.afps_atlas_sequence_parameter_set_id()];

    VERIFY_MIVBITSTREAM(!aau.afps.afps_fixed_camera_model_flag());
    aau.aps = atlas.apsV[atgh.atgh_adaptation_parameter_set_id()];

    aau.geoFrame = m_geoFrameServer(atlasId, sequence_.frameId);
    aau.attrFrame = m_attrFrameServer(atlasId, sequence_.frameId);
  }

  for (const auto &x : onFrame) {
    if (!x(au)) {
      m_stop = true;
    }
  }
}

auto MivDecoder::haveFrame(const VpccUnitHeader &vuh) const -> bool {
  if (mode == Mode::TMC2) {
    return false;
  }
  const auto &sequence_ = sequence(vuh);
  return all_of(cbegin(sequence_.atlas), cend(sequence_.atlas),
                [=](const Atlas &atlas) { return !atlas.atgl.empty(); });
}

// Decoding processes //////////////////////////////////////////////////////////////////////////////

void MivDecoder::decodeVpccPayload(const VpccUnitHeader & /* vuh */,
                                   const monostate & /* payload */) {
  VPCCBITSTREAM_ERROR("V-PCC payload of unknown type");
}

void MivDecoder::decodeVpccPayload(const VpccUnitHeader & /*vuh*/, const VpccParameterSet &vps) {
  const auto id = vps.vps_vpcc_parameter_set_id();

  while (m_vpsV.size() <= id) {
    m_vpsV.emplace_back();
    m_sequenceV.emplace_back();
  }

  m_vpsV[id] = vps;
  m_sequenceV[id] = Sequence{};
  m_sequenceV[id].atlas.resize(vps.vps_atlas_count());

  outputSequence(vps);
}

void MivDecoder::decodeVpccPayload(const VpccUnitHeader &vuh, const AtlasSubBitstream &ad) {
  for (const auto &nu : ad.nal_units()) {
    decodeNalUnit(vuh, nu);
  }
}

void MivDecoder::decodeVpccPayload(const VpccUnitHeader & /*vuh*/,
                                   const VideoSubBitstream & /*vd*/) {
  cout << "WARNING: Ignoring video sub bitstreams in this version of TMIV\n";
}

void MivDecoder::decodeNalUnit(const VpccUnitHeader &vuh, const NalUnit &nu) {
  cout << nu.nal_unit_header();

  if (nu.nal_unit_header().nal_layer_id() != 0) {
    cout << "WARNING: Ignoring NAL unit with nal_layer_id != 0\n";
    return;
  }

  if (NalUnitType::NAL_TRAIL <= nu.nal_unit_header().nal_unit_type() &&
      nu.nal_unit_header().nal_unit_type() < NalUnitType::NAL_ASPS) {
    return parseAtgl(vuh, nu);
  }

  switch (nu.nal_unit_header().nal_unit_type()) {
  case NalUnitType::NAL_ASPS:
    return parseAsps(vuh, nu);
  case NalUnitType::NAL_AFPS:
    return parseAfps(vuh, nu);
  case NalUnitType::NAL_AUD:
    return parseAud(vuh, nu);
  case NalUnitType::NAL_VPCC_AUD:
    return parseVpccAud(vuh, nu);
  case NalUnitType::NAL_EOS:
    return decodeEos(vuh, nu.nal_unit_header());
  case NalUnitType::NAL_EOB:
    return decodeEob(vuh, nu.nal_unit_header());
  case NalUnitType::NAL_FD:
    return decodeFd(vuh, nu.nal_unit_header());
  case NalUnitType::NAL_PREFIX_NSEI:
    return parsePrefixNSei(vuh, nu);
  case NalUnitType::NAL_SUFFIX_NSEI:
    return parseSuffixNSei(vuh, nu);
  case NalUnitType::NAL_PREFIX_ESEI:
    return parsePrefixESei(vuh, nu);
  case NalUnitType::NAL_SUFFIX_ESEI:
    return parseSuffixESei(vuh, nu);
  case NalUnitType::NAL_APS:
    return parseAps(vuh, nu);
  default:
    return decodeUnknownNalUnit(vuh, nu);
  }
}

void MivDecoder::decodeUnknownNalUnit(const VpccUnitHeader & /*vuh*/, const NalUnit &nu) {
  cout << "WARNING: Ignoring NAL unit of unknown type " << nu.nal_unit_header().nal_unit_type()
       << '\n';
}

void MivDecoder::decodeAtgl(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                            AtlasTileGroupLayerRBSP atgl) {
  const auto &atgh = atgl.atlas_tile_group_header();
  auto &x = atlas(vuh);

  if (NalUnitType::NAL_TRAIL <= nuh.nal_unit_type() &&
      nuh.nal_unit_type() < NalUnitType::NAL_BLA_W_LP) {
    VERIFY_VPCCBITSTREAM(nuh.nal_temporal_id() > 0 && atgh.atgh_type() == AtghType::SKIP_TILE_GRP);
    VERIFY_VPCCBITSTREAM(!x.atgl.empty());
    x.atgl.push_back(x.atgl.back());
  }

  if (NalUnitType::NAL_BLA_W_LP <= nuh.nal_unit_type() &&
      nuh.nal_unit_type() < NalUnitType::NAL_ASPS) {
    VERIFY_VPCCBITSTREAM(nuh.nal_temporal_id() == 0 && atgh.atgh_type() == AtghType::I_TILE_GRP);
    x.atgl.push_back(atgl);
  }

  if (haveFrame(vuh)) {
    outputFrame(vuh);
  }
}

// Decoding processes //////////////////////////////////////////////////////////////////////////////

void MivDecoder::decodeAsps(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                            AtlasSequenceParameterSetRBSP asps) {
  VERIFY_VPCCBITSTREAM(nuh.nal_temporal_id() == 0);
  VERIFY_VPCCBITSTREAM(vps(vuh).vps_map_count(vuh.vuh_atlas_id()) - 1 ==
                       asps.asps_map_count_minus1());

  auto &x = atlas(vuh);
  while (x.aspsV.size() <= asps.asps_atlas_sequence_parameter_set_id()) {
    x.aspsV.emplace_back();
  }
  x.aspsV[asps.asps_atlas_sequence_parameter_set_id()] = move(asps);
}

void MivDecoder::decodeAfps(const VpccUnitHeader &vuh, const NalUnitHeader & /*nuh*/,
                            AtlasFrameParameterSetRBSP afps) {
  auto &x = atlas(vuh);
  while (x.afpsV.size() <= afps.afps_atlas_frame_parameter_set_id()) {
    x.afpsV.emplace_back();
  }
  x.afpsV[afps.afps_atlas_frame_parameter_set_id()] = afps;
}

void MivDecoder::decodeAps(const VpccUnitHeader &vuh, const NalUnitHeader & /*nuh*/,
                           AdaptationParameterSetRBSP aps) {
  auto &x = atlas(vuh);
  while (x.apsV.size() <= aps.aps_adaptation_parameter_set_id()) {
    x.apsV.emplace_back();
  }
  x.apsV[aps.aps_adaptation_parameter_set_id()] = aps;
}

void MivDecoder::decodeAud(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                           AccessUnitDelimiterRBSP aud) {
  // There is no normative decoding process associated with the access unit delimiter. Print to
  // prove that we have decoded this NAL unit.
  cout << "Access unit delimiter:\n" << vuh << nuh << aud;
}

void MivDecoder::decodeVpccAud(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                               AccessUnitDelimiterRBSP aud) {
  // There is no normative decoding process associated with the access unit delimiter. Print to
  // prove that we have decoded this NAL unit.
  cout << "V-PCC access unit delimiter:\n" << vuh << nuh << aud;
}

void MivDecoder::decodeEos(const VpccUnitHeader &vuh, const NalUnitHeader &nuh) {
  VERIFY_VPCCBITSTREAM(nuh.nal_temporal_id() == 0);

  // The next NAL unit when present will be an IRAP access unit. Clear state.
  sequence(vuh) = {};
}

void MivDecoder::decodeEob(const VpccUnitHeader &vuh, const NalUnitHeader &nuh) {
  VERIFY_VPCCBITSTREAM(nuh.nal_temporal_id() == 0);

  // TODO(BK): It is unclear what to do with this NAL unit. There could be another V-PCC unit with
  // atlas data. Does that one have to start with a new ASPS? My guess is that it does.
  sequence(vuh) = {};
}

void MivDecoder::decodeFd(const VpccUnitHeader &vuh, const NalUnitHeader &nuh) {
  // There is no normative decoding process associated with the access unit delimiter. Print to
  // prove that we have parsed this NAL unit header.
  cout << "Ignoring filler data:\n" << vuh << nuh;
}

void MivDecoder::decodePrefixNSei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                                  SeiRBSP sei) {
  // Print to prove that we have decoded this NAL unit
  cout << "Prefix non-essential supplemental enhancement information (NSEI):\n"
       << vuh << nuh << sei;
}

void MivDecoder::decodeSuffixNSei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                                  SeiRBSP sei) {
  // Print to prove that we have decoded this NAL unit
  cout << "Suffix non-essential supplemental enhancement information (NSEI):\n"
       << vuh << nuh << sei;
}

void MivDecoder::decodePrefixESei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                                  SeiRBSP sei) {
  // Print to prove that we have decoded this NAL unit
  cout << "Prefix essential supplemental enhancement information (ESEI):\n" << vuh << nuh << sei;
}

void MivDecoder::decodeSuffixESei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                                  SeiRBSP sei) {
  // Print to prove that we have decoded this NAL unit
  cout << "Suffix essential supplemental enhancement information (ESEI):\n" << vuh << nuh << sei;
}

// Parsers /////////////////////////////////////////////////////////////////////////////////////////

void MivDecoder::parseAsps(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAsps(vuh, nu.nal_unit_header(), AtlasSequenceParameterSetRBSP::decodeFrom(stream));
}

void MivDecoder::parseAfps(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAfps(vuh, nu.nal_unit_header(), AtlasFrameParameterSetRBSP::decodeFrom(stream, aspsV(vuh)));
}

void MivDecoder::parseAps(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAps(vuh, nu.nal_unit_header(), AdaptationParameterSetRBSP::decodeFrom(stream));
}

void MivDecoder::parseAtgl(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAtgl(vuh, nu.nal_unit_header(),
             AtlasTileGroupLayerRBSP::decodeFrom(stream, vuh, vps(vuh), aspsV(vuh), afpsV(vuh)));
}

void MivDecoder::parseAud(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAud(vuh, nu.nal_unit_header(), AccessUnitDelimiterRBSP::decodeFrom(stream));
}

void MivDecoder::parseVpccAud(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeVpccAud(vuh, nu.nal_unit_header(), AccessUnitDelimiterRBSP::decodeFrom(stream));
}

void MivDecoder::parsePrefixNSei(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodePrefixNSei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::parseSuffixNSei(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeSuffixNSei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::parsePrefixESei(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodePrefixESei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::parseSuffixESei(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeSuffixESei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

// Access internal decoder state ///////////////////////////////////////////////////////////////////

auto MivDecoder::vps(const VpccUnitHeader &vuh) const -> const VpccParameterSet & {
  VERIFY_VPCCBITSTREAM(vuh.vuh_vpcc_parameter_set_id() < m_vpsV.size());
  return m_vpsV[vuh.vuh_vpcc_parameter_set_id()];
}

auto MivDecoder::sequence(const VpccUnitHeader &vuh) const -> const Sequence & {
  VERIFY_VPCCBITSTREAM(vuh.vuh_vpcc_parameter_set_id() < m_vpsV.size());
  return m_sequenceV[vuh.vuh_vpcc_parameter_set_id()];
}

auto MivDecoder::sequence(const VpccUnitHeader &vuh) -> Sequence & {
  VERIFY_VPCCBITSTREAM(vuh.vuh_vpcc_parameter_set_id() < m_vpsV.size());
  return m_sequenceV[vuh.vuh_vpcc_parameter_set_id()];
}

auto MivDecoder::atlas(const VpccUnitHeader &vuh) const -> const Atlas & {
  if (vuh.vuh_atlas_id() == specialAtlasId) {
    return specialAtlas(vuh);
  }

  const auto &x = sequence(vuh);

  VERIFY_VPCCBITSTREAM(vuh.vuh_atlas_id() < x.atlas.size());
  return x.atlas[vuh.vuh_atlas_id()];
}

auto MivDecoder::atlas(const VpccUnitHeader &vuh) -> Atlas & {
  if (vuh.vuh_atlas_id() == specialAtlasId) {
    return specialAtlas(vuh);
  }

  auto &x = sequence(vuh);

  VERIFY_VPCCBITSTREAM(vuh.vuh_atlas_id() < x.atlas.size());
  return x.atlas[vuh.vuh_atlas_id()];
}

auto MivDecoder::specialAtlas(const VpccUnitHeader &vuh) const -> const Atlas & {
  const auto &x = sequence(vuh);

  VERIFY_MIVBITSTREAM(x.specialAtlas.has_value());
  return *x.specialAtlas;
}

auto MivDecoder::specialAtlas(const VpccUnitHeader &vuh) -> Atlas & {
  auto &x = sequence(vuh);

  VERIFY_MIVBITSTREAM(x.specialAtlas.has_value());
  return *x.specialAtlas;
}

auto MivDecoder::aspsV(const VpccUnitHeader &vuh) const
    -> const vector<AtlasSequenceParameterSetRBSP> & {
  auto &x = atlas(vuh);
  if (x.aspsV.empty()) {
    return specialAtlas(vuh).aspsV;
  }
  return x.aspsV;
}

auto MivDecoder::afpsV(const VpccUnitHeader &vuh) const
    -> const vector<AtlasFrameParameterSetRBSP> & {
  auto &x = atlas(vuh);
  if (x.afpsV.empty()) {
    return specialAtlas(vuh).afpsV;
  }
  return x.afpsV;
}
} // namespace TMIV::MivBitstream
