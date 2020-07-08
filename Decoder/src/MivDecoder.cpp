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

#include <TMIV/Decoder/MivDecoder.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Bytestream.h>
#include <TMIV/MivBitstream/MivDecoderMode.h>

#include <cassert>
#include <istream>
#include <sstream>
#include <thread>
#include <variant>

#include <TMIV/MivBitstream/verify.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::VideoDecoder;

namespace TMIV::Decoder {
// Decoder interface ///////////////////////////////////////////////////////////////////////////////

MivDecoder::MivDecoder(istream &stream)
    : m_stream{stream}, m_ssvh{SampleStreamV3cHeader::decodeFrom(stream)} {}

MivDecoder::~MivDecoder() = default;

void MivDecoder::setGeoFrameServer(GeoFrameServer value) { m_geoFrameServer = move(value); }

void MivDecoder::setAttrFrameServer(AttrFrameServer value) { m_attrFrameServer = move(value); }

void MivDecoder::decode() {
  m_totalGeoVideoDecodingTime = 0.;
  m_totalAttrVideoDecodingTime = 0.;

  auto report = [this]() {
    cout << "Total geometry video sub bitstream decoding time: " << m_totalGeoVideoDecodingTime
         << " s\n";
    cout << "Total attribute video sub bitstream decoding time: " << m_totalAttrVideoDecodingTime
         << " s\n";
  };

  while (!m_stop && !m_stream.eof()) {
    VERIFY_V3CBITSTREAM(m_stream.good());

    const auto ssvu = SampleStreamV3cUnit::decodeFrom(m_stream, m_ssvh);
    VERIFY_V3CBITSTREAM(m_stream.good());

    istringstream substream{ssvu.ssvu_v3c_unit()};
    const auto vu = V3cUnit::decodeFrom(substream, m_vpsV, ssvu.ssvu_v3c_unit_size());
    const auto &vuh = vu.v3c_unit_header();

    decodeV3cPayload(vuh, vu.v3c_payload().payload());

    if (vuh.vuh_unit_type() == VuhUnitType::V3C_AD) {
      while (haveFrame(vuh)) {
        outputFrame(vuh);
        if (m_stop) {
          report();
          return;
        }
      }
    }

    m_stream.peek();
  }

  report();
}

// Decoder output //////////////////////////////////////////////////////////////////////////////////

void MivDecoder::outputSequence(const V3cParameterSet &vps) {
  for (const auto &x : onSequence) {
    if (!x(vps)) {
      m_stop = true;
    }
  }
}

void MivDecoder::outputFrame(const V3cUnitHeader &vuh) {
  assert(haveFrame(vuh) && !m_stop);

  auto au = AccessUnit{};
  au.vps = &vps(vuh);
  outputAtlasData(au);
  outputGeoVideoData(au);
  outputAttrVideoData(au);

  for (const auto &x : onFrame) {
    if (!x(au)) {
      m_stop = true;
    }
  }
}

void MivDecoder::outputAtlasData(AccessUnit &au) {
  auto &sequence_ = m_sequenceV[au.vps->vps_v3c_parameter_set_id()];
  au.frameId = ++sequence_.frameId;
  au.atlas.resize(sequence_.atlas.size());

  for (size_t j = 0; j < au.atlas.size(); ++j) {
    auto &atlas = sequence_.atlas[j];
    auto &aau = au.atlas[j];
    assert(!atlas.frames.empty());

    aau.ath = atlas.frames.front()->ath;
    aau.afps = atlas.afpsV[aau.ath.ath_atlas_frame_parameter_set_id()];
    aau.asps = atlas.aspsV[aau.afps.afps_atlas_sequence_parameter_set_id()];

    aau.viewParamsList = atlas.frames.front()->viewParamsList;
    aau.patchParamsList = atlas.frames.front()->patchParamsList;
    aau.blockToPatchMap = atlas.frames.front()->blockToPatchMap;

    atlas.frames.erase(atlas.frames.begin());
  }
}

auto MivDecoder::haveFrame(const V3cUnitHeader &vuh) const -> bool {
  const auto &sequence_ = sequence(vuh);
  return all_of(cbegin(sequence_.atlas), cend(sequence_.atlas),
                [=](const Atlas &atlas) { return !atlas.frames.empty(); });
}

// Decoding processes //////////////////////////////////////////////////////////////////////////////

void MivDecoder::decodeV3cPayload(const V3cUnitHeader &vuh, const V3cPayload::Payload &payload) {
  switch (vuh.vuh_unit_type()) {
  case VuhUnitType::V3C_VPS:
    return decodeVps(vuh, get<V3cParameterSet>(payload));
  case VuhUnitType::V3C_AD:
    return decodeAsb(vuh, get<AtlasSubBitstream>(payload));
  case VuhUnitType::V3C_AVD:
  case VuhUnitType::V3C_GVD:
  case VuhUnitType::V3C_OVD:
    return;
  default:
    V3CBITSTREAM_ERROR("V3C unit of unknown type");
  }
}

void MivDecoder::decodeVps(const V3cUnitHeader & /* vuh */, const V3cParameterSet &vps) {
  for (uint8_t j = 0; j <= vps.vps_atlas_count_minus1(); ++j) {
    VERIFY_MIVBITSTREAM(!vps.vps_auxiliary_video_present_flag(j));
    VERIFY_MIVBITSTREAM(!vps.vps_occupancy_video_present_flag(j));
    VERIFY_MIVBITSTREAM(vps.vps_geometry_video_present_flag(j));
    LIMITATION(vps.vps_atlas_id(j) == j);
  }

  while (m_vpsV.size() <= vps.vps_v3c_parameter_set_id()) {
    m_vpsV.emplace_back();
    m_sequenceV.emplace_back();
  }

  m_vpsV[vps.vps_v3c_parameter_set_id()] = vps;
  m_sequenceV[vps.vps_v3c_parameter_set_id()] = Sequence{};
  m_sequenceV[vps.vps_v3c_parameter_set_id()].atlas.resize(vps.vps_atlas_count_minus1() + 1U);

  outputSequence(vps);

  if (decodeVideoSubBitstreams(vps)) {
    startGeoVideoDecoders(vps);
    startAttrVideoDecoders(vps);
  }
}

auto MivDecoder::decodeVideoSubBitstreams(const V3cParameterSet &vps) -> bool {
  auto haveVideo = false;
  const auto initialStreamPosition = m_stream.tellg();
  m_stream.peek();

  while (!m_stream.eof()) {
    VERIFY_V3CBITSTREAM(m_stream.good());

    const auto ssvu = SampleStreamV3cUnit::decodeFrom(m_stream, m_ssvh);
    VERIFY_V3CBITSTREAM(m_stream.good());

    istringstream substream{ssvu.ssvu_v3c_unit().substr(0, 4)};
    const auto vuh = V3cUnitHeader::decodeFrom(substream, m_vpsV);

    if (vuh.vuh_v3c_parameter_set_id() == vps.vps_v3c_parameter_set_id()) {
      if (vuh.vuh_unit_type() == VuhUnitType::V3C_GVD) {
        haveVideo = true;
        const auto j = vps.atlasIdxOf(vuh.vuh_atlas_id());
        sequence(vuh).atlas[j].geoVideoData += ssvu.ssvu_v3c_unit().substr(4);
      }
      if (vuh.vuh_unit_type() == VuhUnitType::V3C_AVD) {
        haveVideo = true;
        const auto j = vps.atlasIdxOf(vuh.vuh_atlas_id());
        sequence(vuh).atlas[j].attrVideoData += ssvu.ssvu_v3c_unit().substr(4);
      }
    }

    m_stream.peek();
  }

  m_stream.seekg(initialStreamPosition);
  m_stream.clear();
  return haveVideo;
}

void MivDecoder::startGeoVideoDecoders(const V3cParameterSet &vps) {
  auto &sequence_ = m_sequenceV[vps.vps_v3c_parameter_set_id()];
  const auto codecGroupIdc = vps.profile_tier_level().ptl_profile_codec_group_idc();
  const auto t0 = clock();

  for (uint8_t j = 0; j <= vps.vps_atlas_count_minus1(); ++j) {
    if (vps.vps_geometry_video_present_flag(j)) {
      sequence_.atlas[j].geoVideoServer = make_unique<VideoServer>(
          IVideoDecoder::create(codecGroupIdc), sequence_.atlas[j].geoVideoData);
      sequence_.atlas[j].geoVideoServer->wait();
    }
  }

  // Measure video decoding time
  const auto dt = double(clock() - t0) / CLOCKS_PER_SEC;
  cout << "Time taken for decoding all geometry video sub bitstreams, first frame: " << dt
       << " s\n";
  m_totalGeoVideoDecodingTime += dt;
}

void MivDecoder::startAttrVideoDecoders(const V3cParameterSet &vps) {
  auto &sequence_ = m_sequenceV[vps.vps_v3c_parameter_set_id()];
  const auto codecGroupIdc = vps.profile_tier_level().ptl_profile_codec_group_idc();
  const auto t0 = clock();

  for (uint8_t j = 0; j <= vps.vps_atlas_count_minus1(); ++j) {
    if (vps.vps_attribute_video_present_flag(j)) {
      sequence_.atlas[j].attrVideoServer = make_unique<VideoServer>(
          IVideoDecoder::create(codecGroupIdc), sequence_.atlas[j].attrVideoData);
      sequence_.atlas[j].attrVideoServer->wait();
    }
  }

  // Measure video decoding time
  const auto dt = double(clock() - t0) / CLOCKS_PER_SEC;
  cout << "Time taken for decoding all attribute video sub bitstreams, first frame: " << dt
       << " s\n";
  m_totalAttrVideoDecodingTime += dt;
}

void MivDecoder::outputGeoVideoData(AccessUnit &au) {
  auto &sequence_ = m_sequenceV[au.vps->vps_v3c_parameter_set_id()];
  const auto t0 = clock();

  for (size_t j = 0; j < au.atlas.size(); ++j) {
    auto &aau = au.atlas[j];
    auto &atlas_ = sequence_.atlas[j];

    // Get a geometry frame in-band or out-of-band
    if (atlas_.geoVideoServer) {
      auto frame = atlas_.geoVideoServer->getFrame();
      VERIFY_MIVBITSTREAM(frame);
      aau.decGeoFrame = frame->as<YUV400P10>();
      atlas_.geoVideoServer->wait();
    } else if (m_geoFrameServer) {
      aau.decGeoFrame =
          m_geoFrameServer(uint8_t(j), sequence_.frameId, aau.decGeoFrameSize(*au.vps));
    } else {
      MIVBITSTREAM_ERROR("Out-of-band geometry video data but no frame server provided");
    }
  }

  // Measure video decoding time
  const auto dt = double(clock() - t0) / CLOCKS_PER_SEC;
  cout << "Time taken for decoding all geometry video sub bitstreams, one frame: " << dt << " s\n";
  m_totalGeoVideoDecodingTime += dt;
}

void MivDecoder::outputAttrVideoData(AccessUnit &au) {
  auto &sequence_ = m_sequenceV[au.vps->vps_v3c_parameter_set_id()];
  const auto t0 = clock();

  for (size_t j = 0; j < au.atlas.size(); ++j) {
    auto &atlas = sequence_.atlas[j];
    auto &aau = au.atlas[j];

    if (au.vps->vps_attribute_video_present_flag(uint8_t(j))) {
      // Get a texture frame in-band or out-of-band
      if (atlas.attrVideoServer) {
        auto frame = atlas.attrVideoServer->getFrame();
        VERIFY_MIVBITSTREAM(frame);
        aau.attrFrame = frame->as<YUV444P10>();
        atlas.attrVideoServer->wait();
      } else if (m_attrFrameServer) {
        aau.attrFrame = m_attrFrameServer(uint8_t(j), sequence_.frameId, aau.frameSize());
      } else {
        MIVBITSTREAM_ERROR("Out-of-band attribute video data but no frame server provided");
      }
    } else {
      // No texture: pass out a neutral gray frame
      aau.attrFrame.resize(aau.asps.asps_frame_width(), aau.asps.asps_frame_height());
      aau.attrFrame.fillNeutral();
      VERIFY_MIVBITSTREAM(aau.decGeoFrameSize(*au.vps) == aau.frameSize());
    }
  }

  // Measure video decoding time
  const auto dt = double(clock() - t0) / CLOCKS_PER_SEC;
  cout << "Time taken for decoding all attribute video sub bitstreams, one frame: " << dt << " s\n";
  m_totalAttrVideoDecodingTime += dt;
}

void MivDecoder::decodeAsb(const V3cUnitHeader &vuh, const AtlasSubBitstream &asb) {
  for (const auto &nu : asb.nal_units()) {
    decodeNalUnit(vuh, nu);
  }
}

void MivDecoder::decodeNalUnit(const V3cUnitHeader &vuh, const NalUnit &nu) {
  if (nu.nal_unit_header().nal_layer_id() != 0) {
    cout << "WARNING: Ignoring NAL unit with nal_layer_id != 0\n";
    return;
  }

  if (NalUnitType::NAL_TRAIL_N <= nu.nal_unit_header().nal_unit_type() &&
      nu.nal_unit_header().nal_unit_type() < NalUnitType::NAL_ASPS) {
    return parseAtl(vuh, nu);
  }

  switch (nu.nal_unit_header().nal_unit_type()) {
  case NalUnitType::NAL_ASPS:
    return parseAsps(vuh, nu);
  case NalUnitType::NAL_AFPS:
    return parseAfps(vuh, nu);
  case NalUnitType::NAL_AUD:
    return parseAud(vuh, nu);
  case NalUnitType::NAL_V3C_AUD:
    return parseV3cAud(vuh, nu);
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
  case NalUnitType::NAL_AAPS:
    return parseAaps(vuh, nu);
  default:
    return decodeUnknownNalUnit(vuh, nu);
  }
}

void MivDecoder::decodeUnknownNalUnit(const V3cUnitHeader & /*vuh*/, const NalUnit &nu) {
  cout << "WARNING: Ignoring NAL unit of unknown type " << nu.nal_unit_header().nal_unit_type()
       << '\n';
}

void MivDecoder::decodeAtl(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                           const AtlasTileLayerRBSP &atl) {
  const auto &ath = atl.atlas_tile_header();
  auto &x = atlas(vuh);

  if (mode != MivDecoderMode::TMC2 && (NalUnitType::NAL_TRAIL_N <= nuh.nal_unit_type() &&
                                       nuh.nal_unit_type() < NalUnitType::NAL_BLA_W_LP)) {
    VERIFY_V3CBITSTREAM(nuh.nal_temporal_id_plus1() - 1 > 0 &&
                        ath.ath_type() == AthType::SKIP_TILE);
    VERIFY_V3CBITSTREAM(x.intraFrame);
    VERIFY_V3CBITSTREAM(x.intraFrame->ath.ath_atlas_frame_parameter_set_id() ==
                        ath.ath_atlas_frame_parameter_set_id());
    VERIFY_V3CBITSTREAM(x.intraFrame->ath.ath_atlas_adaptation_parameter_set_id() ==
                        ath.ath_atlas_adaptation_parameter_set_id());

    // Shallow copy of the intra atlas frame
    x.frames.push_back(x.intraFrame);
  }

  if (mode == MivDecoderMode::TMC2 || (NalUnitType::NAL_BLA_W_LP <= nuh.nal_unit_type() &&
                                       nuh.nal_unit_type() < NalUnitType::NAL_ASPS)) {
    VERIFY_V3CBITSTREAM(nuh.nal_temporal_id_plus1() - 1 == 0 && ath.ath_type() == AthType::I_TILE);

    auto frame = make_shared<Atlas::Frame>();
    frame->ath = ath;

    const auto noAaps = AtlasAdaptationParameterSetRBSP{};
    const auto &aaps = mode == MivDecoderMode::TMC2
                           ? noAaps
                           : aapsV(vuh)[ath.ath_atlas_adaptation_parameter_set_id()];

    if (aaps.aaps_miv_extension_flag()) {
      const auto &aame = aaps.aaps_miv_extension();
      const auto &mvpl = aame.miv_view_params_list();
      frame->viewParamsList = decodeMvpl(mvpl);
    }

    const auto &afps = afpsV(vuh)[ath.ath_atlas_frame_parameter_set_id()];
    const auto &asps = aspsV(vuh)[afps.afps_atlas_sequence_parameter_set_id()];
    const auto &atdu = atl.atlas_tile_data_unit();

    frame->patchParamsList = decodeAtdu(atdu, atl.atlas_tile_header(), asps);
    frame->blockToPatchMap = decodeBlockToPatchMap(atdu, asps);

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
      x[viewId].pp = mvpl.pruning_parent(viewId);
    }
  }
  return ViewParamsList{x};
}

auto MivDecoder::decodeAtdu(const AtlasTileDataUnit &atdu, const AtlasTileHeader &ath,
                            const AtlasSequenceParameterSetRBSP &asps) -> PatchParamsList {
  auto x = vector<PatchParams>(atdu.atduTotalNumberOfPatches());

  atdu.visit([&](size_t p, AtduPatchMode /* unused */, const PatchInformationData &pid) {
    const auto &pdu = pid.patch_data_unit();
    const auto k = asps.asps_log2_patch_packing_block_size();

    x[p].pduOrientationIndex(pdu.pdu_orientation_index());
    x[p].pdu2dPos({int(pdu.pdu_2d_pos_x() << k), int(pdu.pdu_2d_pos_y() << k)});
    x[p].pdu2dSize(
        {int((pdu.pdu_2d_size_x_minus1() + 1U) << k), int((pdu.pdu_2d_size_y_minus1() + 1U) << k)});
    x[p].pduViewPos({pdu.pdu_view_pos_x(), pdu.pdu_view_pos_y()});
    x[p].pduDepthStart(pdu.pdu_depth_start() << ath.ath_pos_min_z_quantizer());
    x[p].pduViewId(pdu.pdu_projection_id());

    if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
      x[p].pduDepthEnd(pdu.pdu_depth_end() << ath.ath_pos_delta_max_z_quantizer());
    }
    if (asps.asps_miv_extension_flag()) {
      x[p].pduEntityId(pdu.pdu_miv_extension().pdu_entity_id());

      if (asps.asps_miv_extension().asme_depth_occ_threshold_flag()) {
        x[p].pduDepthOccMapThreshold(pdu.pdu_miv_extension().pdu_depth_occ_threshold());
      }
    }
  });
  return x;
}

auto MivDecoder::decodeBlockToPatchMap(const AtlasTileDataUnit &atdu,
                                       const AtlasSequenceParameterSetRBSP &asps)
    -> Common::BlockToPatchMap {
  auto result =
      BlockToPatchMap{asps.asps_frame_width() >> asps.asps_log2_patch_packing_block_size(),
                      asps.asps_frame_height() >> asps.asps_log2_patch_packing_block_size()};
  fill(begin(result.getPlane(0)), end(result.getPlane(0)), unusedPatchId);

  atdu.visit([&result](size_t p, AtduPatchMode /* unused */, const PatchInformationData &pid) {
    const auto &pdu = pid.patch_data_unit();
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

void MivDecoder::decodeAsps(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                            AtlasSequenceParameterSetRBSP asps) {
  VERIFY_V3CBITSTREAM(nuh.nal_temporal_id_plus1() - 1 == 0);
  auto &x = atlas(vuh);
  while (x.aspsV.size() <= asps.asps_atlas_sequence_parameter_set_id()) {
    x.aspsV.emplace_back();
  }
  x.aspsV[asps.asps_atlas_sequence_parameter_set_id()] = move(asps);
}

void MivDecoder::decodeAfps(const V3cUnitHeader &vuh, const NalUnitHeader & /*nuh*/,
                            const AtlasFrameParameterSetRBSP &afps) {
  auto &x = atlas(vuh);
  while (x.afpsV.size() <= afps.afps_atlas_frame_parameter_set_id()) {
    x.afpsV.emplace_back();
  }
  x.afpsV[afps.afps_atlas_frame_parameter_set_id()] = afps;
}

void MivDecoder::decodeAaps(const V3cUnitHeader &vuh, const NalUnitHeader & /*nuh*/,
                            const AtlasAdaptationParameterSetRBSP &aaps) {
  auto &x = atlas(vuh);
  while (x.aapsV.size() <= aaps.aaps_atlas_adaptation_parameter_set_id()) {
    x.aapsV.emplace_back();
  }
  x.aapsV[aaps.aaps_atlas_adaptation_parameter_set_id()] = aaps;
}

void MivDecoder::decodeAud(const V3cUnitHeader & /* vuh */, const NalUnitHeader & /* nuh */,
                           AccessUnitDelimiterRBSP aud) {}

void MivDecoder::decodeEos(const V3cUnitHeader &vuh, const NalUnitHeader &nuh) {
  VERIFY_V3CBITSTREAM(nuh.nal_temporal_id_plus1() - 1 == 0);

  // The next NAL unit when present will be an IRAP access unit. Clear state.
  sequence(vuh) = {};
}

void MivDecoder::decodeEob(const V3cUnitHeader &vuh, const NalUnitHeader &nuh) {
  VERIFY_V3CBITSTREAM(nuh.nal_temporal_id_plus1() - 1 == 0);
  sequence(vuh) = {};
}

void MivDecoder::decodeSei(const V3cUnitHeader &vuh, const NalUnitHeader &nuh, const SeiRBSP &sei) {
  for (const auto &message : sei.messages()) {
    decodeSeiMessage(vuh, nuh, message);
  }
}

void MivDecoder::decodeSeiMessage(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                                  const SeiMessage &message) {
  cout << "WARNING: Ignoring SEI message\n";
}

// Parsers /////////////////////////////////////////////////////////////////////////////////////////

void MivDecoder::parseAsps(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAsps(vuh, nu.nal_unit_header(),
             AtlasSequenceParameterSetRBSP::decodeFrom(stream, vuh, vps(vuh)));
}

void MivDecoder::parseAfps(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAfps(vuh, nu.nal_unit_header(), AtlasFrameParameterSetRBSP::decodeFrom(stream, aspsV(vuh)));
}

void MivDecoder::parseAaps(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAaps(vuh, nu.nal_unit_header(),
             AtlasAdaptationParameterSetRBSP::decodeFrom(stream, vps(vuh)));
}

void MivDecoder::parseAtl(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAtl(vuh, nu.nal_unit_header(),
            AtlasTileLayerRBSP::decodeFrom(stream, vuh, vps(vuh), aspsV(vuh), afpsV(vuh)));
}

void MivDecoder::parseAud(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAud(vuh, nu.nal_unit_header(), AccessUnitDelimiterRBSP::decodeFrom(stream));
}

void MivDecoder::parseV3cAud(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAud(vuh, nu.nal_unit_header(), AccessUnitDelimiterRBSP::decodeFrom(stream));
}

void MivDecoder::parsePrefixNSei(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeSei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::parseSuffixNSei(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeSei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::parsePrefixESei(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeSei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

void MivDecoder::parseSuffixESei(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeSei(vuh, nu.nal_unit_header(), SeiRBSP::decodeFrom(stream));
}

// Access internal decoder state ///////////////////////////////////////////////////////////////////

auto MivDecoder::vps(const V3cUnitHeader &vuh) const -> const V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vuh.vuh_v3c_parameter_set_id() < m_vpsV.size());
  return m_vpsV[vuh.vuh_v3c_parameter_set_id()];
}

auto MivDecoder::sequence(const V3cUnitHeader &vuh) const -> const Sequence & {
  VERIFY_V3CBITSTREAM(vuh.vuh_v3c_parameter_set_id() < m_vpsV.size());
  return m_sequenceV[vuh.vuh_v3c_parameter_set_id()];
}

auto MivDecoder::sequence(const V3cUnitHeader &vuh) -> Sequence & {
  VERIFY_V3CBITSTREAM(vuh.vuh_v3c_parameter_set_id() < m_vpsV.size());
  return m_sequenceV[vuh.vuh_v3c_parameter_set_id()];
}

auto MivDecoder::atlas(const V3cUnitHeader &vuh) const -> const Atlas & {
  if (vuh.vuh_atlas_id() == specialAtlasId) {
    return specialAtlas(vuh);
  }
  const auto &x = sequence(vuh);
  const auto j = vps(vuh).atlasIdxOf(vuh.vuh_atlas_id());
  return x.atlas[j];
}

auto MivDecoder::atlas(const V3cUnitHeader &vuh) -> Atlas & {
  if (vuh.vuh_atlas_id() == specialAtlasId) {
    return specialAtlas(vuh);
  }
  auto &x = sequence(vuh);
  const auto j = vps(vuh).atlasIdxOf(vuh.vuh_atlas_id());
  return x.atlas[j];
}

auto MivDecoder::specialAtlas(const V3cUnitHeader &vuh) const -> const Atlas & {
  return sequence(vuh).specialAtlas;
}

auto MivDecoder::specialAtlas(const V3cUnitHeader &vuh) -> Atlas & {
  return sequence(vuh).specialAtlas;
}

auto MivDecoder::aspsV(const V3cUnitHeader &vuh) const
    -> const vector<AtlasSequenceParameterSetRBSP> & {
  const auto &x = atlas(vuh);
  if (x.aspsV.empty()) {
    return specialAtlas(vuh).aspsV;
  }
  return x.aspsV;
}

auto MivDecoder::afpsV(const V3cUnitHeader &vuh) const
    -> const vector<AtlasFrameParameterSetRBSP> & {
  const auto &x = atlas(vuh);
  if (x.afpsV.empty()) {
    return specialAtlas(vuh).afpsV;
  }
  return x.afpsV;
}

auto MivDecoder::aapsV(const V3cUnitHeader &vuh) const
    -> const vector<AtlasAdaptationParameterSetRBSP> & {
  const auto &x = atlas(vuh);
  if (x.aapsV.empty()) {
    return specialAtlas(vuh).aapsV;
  }
  return x.aapsV;
}
} // namespace TMIV::Decoder
