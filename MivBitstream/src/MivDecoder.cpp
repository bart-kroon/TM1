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
      m_attrFrameServer{move(attrFrameServer)}, m_ssvh{sampleStreamVpccHeader(stream)} {
  cout << "=== Sample stream V-PCC header " << string(100 - 31, '=') << '\n'
       << m_ssvh << string(100, '=') << "\n"
       << endl;
}

auto MivDecoder::decodeVpccUnit() -> bool {
  cout << "\n=== V-PCC unit " << string(100 - 15, '=') << '\n';
  VERIFY_VPCCBITSTREAM(m_stream.good());

  const auto ssvu = SampleStreamVpccUnit::decodeFrom(m_stream, m_ssvh);
  cout << ssvu;
  VERIFY_VPCCBITSTREAM(m_stream.good());

  istringstream substream{ssvu.ssvu_vpcc_unit()};
  const auto vu = VpccUnit::decodeFrom(substream, m_vpsV, ssvu.ssvu_vpcc_unit_size());
  cout << vu;

  const auto &vuh = vu.vpcc_unit_header();
  decodeVpccPayload(vuh, vu.vpcc_payload().payload());
  cout << string(100, '=') << "\n" << endl;

  if (vuh.vuh_unit_type() == VuhUnitType::VPCC_AD) {
    while (haveFrame(vuh)) {
      outputFrame(vuh);
      if (m_stop) {
        return false;
      }
    }
  }

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
  assert(haveFrame(vuh) && !m_stop);
  AccessUnit au;
  au.vps = &vps(vuh);

  auto &sequence_ = sequence(vuh);
  au.frameId = ++sequence_.frameId;
  au.atlas.resize(sequence_.atlas.size());

  for (size_t atlasId = 0; atlasId < au.atlas.size(); ++atlasId) {
    auto &atlas = sequence_.atlas[atlasId];
    auto &aau = au.atlas[atlasId];
    assert(!atlas.frames.empty());

    aau.atgh = atlas.frames.front()->atgh;
    aau.afps = atlas.afpsV[aau.atgh.atgh_atlas_frame_parameter_set_id()];
    aau.asps = atlas.aspsV[aau.afps.afps_atlas_sequence_parameter_set_id()];

    aau.viewParamsList = atlas.frames.front()->viewParamsList;
    aau.patchParamsList = atlas.frames.front()->patchParamsList;
    aau.blockToPatchMap = atlas.frames.front()->blockToPatchMap;

    if (au.vps->attribute_information(uint8_t(atlasId)).ai_attribute_count() >= 1 &&
        au.vps->attribute_information(uint8_t(atlasId)).ai_attribute_type_id(0) ==
            AiAttributeTypeId::ATTR_TEXTURE) {
      aau.attrFrame = m_attrFrameServer(uint8_t(atlasId), sequence_.frameId, aau.frameSize());
    } else {
      aau.attrFrame.resize(aau.asps.asps_frame_width(), aau.asps.asps_frame_height());
      aau.attrFrame.fillNeutral();
	  VERIFY_MIVBITSTREAM(aau.decGeoFrameSize(*au.vps) == aau.frameSize());
    }

    aau.decGeoFrame =
        m_geoFrameServer(uint8_t(atlasId), sequence_.frameId, aau.decGeoFrameSize(*au.vps));

    atlas.frames.erase(atlas.frames.begin());
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
                [=](const Atlas &atlas) { return !atlas.frames.empty(); });
}

// Decoding processes //////////////////////////////////////////////////////////////////////////////

void MivDecoder::decodeVpccPayload(const VpccUnitHeader &vuh, const VpccPayload::Payload &payload) {
  switch (vuh.vuh_unit_type()) {
  case VuhUnitType::VPCC_VPS:
    return decodeVps(vuh, get<VpccParameterSet>(payload));
  case VuhUnitType::VPCC_AD:
    return decodeAsb(vuh, get<AtlasSubBitstream>(payload));
  case VuhUnitType::VPCC_AVD:
  case VuhUnitType::VPCC_GVD:
  case VuhUnitType::VPCC_OVD:
    cout << "WARNING: Ignoring video sub bitstreams in this version of "
            "TMIV\n";
    return;
  default:
    VPCCBITSTREAM_ERROR("V-PCC unit of unknown type");
  }
}

void MivDecoder::decodeVps(const VpccUnitHeader & /* vuh */, const VpccParameterSet &vps) {
  const auto id = vps.vps_vpcc_parameter_set_id();

  while (m_vpsV.size() <= id) {
    m_vpsV.emplace_back();
    m_sequenceV.emplace_back();
  }

  m_vpsV[id] = vps;
  m_sequenceV[id] = Sequence{};
  m_sequenceV[id].atlas.resize(vps.vps_atlas_count_minus1() + 1U);

  outputSequence(vps);
}

void MivDecoder::decodeAsb(const VpccUnitHeader &vuh, const AtlasSubBitstream &asb) {
  for (const auto &nu : asb.nal_units()) {
    decodeNalUnit(vuh, nu);
  }
}

void MivDecoder::decodeNalUnit(const VpccUnitHeader &vuh, const NalUnit &nu) {
  cout << "--- NAL unit " << string(100 - 13, '-') << '\n' << nu;

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
    return;
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
                            const AtlasTileGroupLayerRBSP &atgl) {
  cout << atgl;

  const auto &atgh = atgl.atlas_tile_group_header();
  auto &x = atlas(vuh);

  if (NalUnitType::NAL_TRAIL <= nuh.nal_unit_type() &&
      nuh.nal_unit_type() < NalUnitType::NAL_BLA_W_LP) {
    VERIFY_VPCCBITSTREAM(nuh.nal_temporal_id_plus1() - 1 > 0 &&
                         atgh.atgh_type() == AtghType::SKIP_TILE_GRP);
    VERIFY_VPCCBITSTREAM(x.intraFrame);
    VERIFY_VPCCBITSTREAM(x.intraFrame->atgh.atgh_atlas_frame_parameter_set_id() ==
                         atgh.atgh_atlas_frame_parameter_set_id());
    VERIFY_VPCCBITSTREAM(x.intraFrame->atgh.atgh_adaptation_parameter_set_id() ==
                         atgh.atgh_adaptation_parameter_set_id());

    // Shallow copy of the intra atlas frame
    x.frames.push_back(x.intraFrame);
  }

  if (NalUnitType::NAL_BLA_W_LP <= nuh.nal_unit_type() &&
      nuh.nal_unit_type() < NalUnitType::NAL_ASPS) {
    VERIFY_VPCCBITSTREAM(nuh.nal_temporal_id_plus1() - 1 == 0 &&
                         atgh.atgh_type() == AtghType::I_TILE_GRP);

    auto frame = make_shared<Atlas::Frame>();
    frame->atgh = atgh;

    const auto &aps = apsV(vuh)[atgh.atgh_adaptation_parameter_set_id()];
    const auto &mvpl = aps.miv_view_params_list();

    frame->viewParamsList = decodeMvpl(mvpl);

    const auto &afps = afpsV(vuh)[atgh.atgh_atlas_frame_parameter_set_id()];
    const auto &asps = aspsV(vuh)[afps.afps_atlas_sequence_parameter_set_id()];
    const auto &atgdu = atgl.atlas_tile_group_data_unit();

    frame->patchParamsList = decodeAtgdu(atgdu, asps);
    frame->blockToPatchMap = decodeBlockToPatchMap(atgdu, asps);

    x.frames.push_back(frame);
    x.intraFrame = frame;
  }
}

auto MivDecoder::decodeMvpl(const MivViewParamsList &mvpl) -> ViewParamsList {
  auto x = vector<ViewParams>(mvpl.mvp_num_views_minus1() + 1);

  for (uint16_t viewId = 0; viewId <= mvpl.mvp_num_views_minus1(); ++viewId) {
    x[viewId].ce = mvpl.camera_extrinsics(viewId);
    x[viewId].ci = mvpl.camera_intrinsics(viewId);
    x[viewId].dq = mvpl.depth_quantization(viewId);

    if (mvpl.mvp_pruning_graph_params_present_flag()) {
      x[viewId].pc = mvpl.pruning_children(viewId);
    }
  }
  return ViewParamsList{x};
}

auto MivDecoder::decodeAtgdu(const AtlasTileGroupDataUnit &atgdu,
                             const AtlasSequenceParameterSetRBSP &asps) -> PatchParamsList {
  auto x = vector<PatchParams>(atgdu.atgduTotalNumberOfPatches());

  atgdu.visit([&](size_t p, AtgduPatchMode /* unused */, const PatchInformationData &pid) {
    auto &pdu = pid.patch_data_unit();
    const auto k = asps.asps_log2_patch_packing_block_size();

    x[p].pduOrientationIndex(pdu.pdu_orientation_index());
    x[p].pdu2dPos({int(pdu.pdu_2d_pos_x() << k), int(pdu.pdu_2d_pos_y() << k)});
    x[p].pdu2dSize(
        {int((pdu.pdu_2d_size_x_minus1() + 1U) << k), int((pdu.pdu_2d_size_y_minus1() + 1U) << k)});
    x[p].pduViewPos({pdu.pdu_view_pos_x(), pdu.pdu_view_pos_y()});
    x[p].pduDepthStart(pdu.pdu_depth_start());
    x[p].pduViewId(pdu.pdu_view_id());

    if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
      x[p].pduDepthEnd(pdu.pdu_depth_end());
    }

    x[p].pduEntityId(pdu.pdu_entity_id());

    if (asps.miv_atlas_sequence_params().masp_depth_occ_map_threshold_flag()) {
      x[p].pduDepthOccMapThreshold(pdu.pdu_depth_occ_map_threshold());
    }
  });
  return x;
}

auto MivDecoder::decodeBlockToPatchMap(const AtlasTileGroupDataUnit &atgdu,
                                       const AtlasSequenceParameterSetRBSP &asps)
    -> Common::BlockToPatchMap {
  auto result =
      BlockToPatchMap{asps.asps_frame_width() >> asps.asps_log2_patch_packing_block_size(),
                      asps.asps_frame_height() >> asps.asps_log2_patch_packing_block_size()};
  fill(begin(result.getPlane(0)), end(result.getPlane(0)), unusedPatchId);

  atgdu.visit([&result](size_t p, AtgduPatchMode /* unused */, const PatchInformationData &pid) {
    auto &pdu = pid.patch_data_unit();
    const auto first = Vec2i{pdu.pdu_2d_pos_x(), pdu.pdu_2d_pos_y()};
    const auto last = first + Vec2i{pdu.pdu_2d_size_x_minus1(), pdu.pdu_2d_size_y_minus1()};

    for (int y = first.y(); y <= last.y(); ++y) {
      for (int x = first.x(); x <= last.x(); ++x) {
        result.getPlane(0)(y, x) = uint16_t(p);
      }
    }
  });
  return result;
}

void MivDecoder::decodeAsps(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                            AtlasSequenceParameterSetRBSP asps) {
  VERIFY_VPCCBITSTREAM(nuh.nal_temporal_id_plus1() - 1 == 0);

  cout << asps;

  auto &x = atlas(vuh);
  while (x.aspsV.size() <= asps.asps_atlas_sequence_parameter_set_id()) {
    x.aspsV.emplace_back();
  }
  x.aspsV[asps.asps_atlas_sequence_parameter_set_id()] = move(asps);
}

void MivDecoder::decodeAfps(const VpccUnitHeader &vuh, const NalUnitHeader & /*nuh*/,
                            AtlasFrameParameterSetRBSP afps) {
  cout << afps;

  auto &x = atlas(vuh);
  while (x.afpsV.size() <= afps.afps_atlas_frame_parameter_set_id()) {
    x.afpsV.emplace_back();
  }
  x.afpsV[afps.afps_atlas_frame_parameter_set_id()] = afps;
}

void MivDecoder::decodeAps(const VpccUnitHeader &vuh, const NalUnitHeader & /*nuh*/,
                           const AdaptationParameterSetRBSP &aps) {
  cout << aps;

  auto &x = atlas(vuh);
  while (x.apsV.size() <= aps.aps_adaptation_parameter_set_id()) {
    x.apsV.emplace_back();
  }
  x.apsV[aps.aps_adaptation_parameter_set_id()] = aps;
}

void MivDecoder::decodeAud(const VpccUnitHeader & /* vuh */, const NalUnitHeader & /* nuh */,
                           AccessUnitDelimiterRBSP aud) {
  cout << aud;
}

void MivDecoder::decodeEos(const VpccUnitHeader &vuh, const NalUnitHeader &nuh) {
  VERIFY_VPCCBITSTREAM(nuh.nal_temporal_id_plus1() - 1 == 0);

  // The next NAL unit when present will be an IRAP access unit. Clear state.
  sequence(vuh) = {};
}

void MivDecoder::decodeEob(const VpccUnitHeader &vuh, const NalUnitHeader &nuh) {
  VERIFY_VPCCBITSTREAM(nuh.nal_temporal_id_plus1() - 1 == 0);

  // TODO(BK): It is unclear what to do with this NAL unit. There could be another V-PCC unit with
  // atlas data. Does that one have to start with a new ASPS? My guess is that it does.
  sequence(vuh) = {};
}

void MivDecoder::decodeSei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                           const SeiRBSP &sei) {
  for (auto &message : sei.messages()) {
    decodeSeiMessage(vuh, nuh, message);
  }
}

void MivDecoder::decodeSeiMessage(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                                  const SeiMessage &message) {
  cout << message;

  switch (message.payloadType()) {
  case PayloadType::viewing_space_handling:
    return parseViewingSpaceHandlingSei(vuh, nuh, message);
  default:
    cout << "WARNING: Ignoring SEI message\n";
  }
}

void MivDecoder::decodeViewingSpaceHandling(const VpccUnitHeader & /* vuh */,
                                            const NalUnitHeader & /* nuh */,
                                            const ViewingSpaceHandling &vh) {
  cout << vh;
}

// Parsers /////////////////////////////////////////////////////////////////////////////////////////

void MivDecoder::parseAsps(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAsps(vuh, nu.nal_unit_header(),
             AtlasSequenceParameterSetRBSP::decodeFrom(stream, vuh, vps(vuh)));
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
  decodeAud(vuh, nu.nal_unit_header(), AccessUnitDelimiterRBSP::decodeFrom(stream));
}

void MivDecoder::parsePrefixNSei(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeSei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::parseSuffixNSei(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeSei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::parsePrefixESei(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeSei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::parseSuffixESei(const VpccUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeSei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::parseViewingSpaceHandlingSei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                                              const SeiMessage &message) {
  istringstream stream{message.payload()};
  InputBitstream bitstream{stream};
  decodeViewingSpaceHandling(vuh, nuh, ViewingSpaceHandling::decodeFrom(bitstream));
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
  return sequence(vuh).specialAtlas;
}

auto MivDecoder::specialAtlas(const VpccUnitHeader &vuh) -> Atlas & {
  return sequence(vuh).specialAtlas;
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

auto MivDecoder::apsV(const VpccUnitHeader &vuh) const
    -> const vector<AdaptationParameterSetRBSP> & {
  auto &x = atlas(vuh);
  if (x.apsV.empty()) {
    return specialAtlas(vuh).apsV;
  }
  return x.apsV;
}
} // namespace TMIV::MivBitstream
