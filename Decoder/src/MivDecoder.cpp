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
V3cSampleStreamDecoder::V3cSampleStreamDecoder(std::istream &stream)
    : m_stream{stream}, m_ssvh{SampleStreamV3cHeader::decodeFrom(stream)} {
  VERIFY_V3CBITSTREAM(m_stream.good());
}

auto V3cSampleStreamDecoder::operator()() -> std::optional<V3cUnit> {
  m_stream.peek();
  if (m_stream.eof()) {
    return {};
  }
  auto ssvu = SampleStreamV3cUnit::decodeFrom(m_stream, m_ssvh);
  istringstream substream{ssvu.ssvu_v3c_unit()};
  return V3cUnit::decodeFrom(substream, ssvu.ssvu_v3c_unit_size());
}

V3cUnitBuffer::V3cUnitBuffer(V3cUnitSource source) : m_source{source} {}

auto V3cUnitBuffer::operator()(const V3cUnitHeader &vuh) -> std::optional<V3cUnit> {
  auto i = m_buffer.begin();

  for (;;) {
    if (i == m_buffer.end()) {
      if (auto vu = m_source()) {
        m_buffer.push_back(std::move(*vu));
        i = std::prev(m_buffer.end());
      } else {
        return {};
      }
    }
    if (i->v3c_unit_header() == vuh) {
      auto vu = std::move(*i);
      i = m_buffer.erase(i);
      return vu;
    }
    if (vuh.vuh_unit_type() == VuhUnitType::V3C_VPS) {
      std::cout << "WARNING: Skipping V3C unit.\n";
      i = m_buffer.erase(i);
    } else {
      ++i;
    }
  }
}

MivDecoder::MivDecoder(V3cUnitSource source) : m_inputBuffer{source} {}

MivDecoder::~MivDecoder() {
  if (m_totalGeoVideoDecodingTime > 0.) {
    cout << "Total geometry video sub bitstream decoding time: " << m_totalGeoVideoDecodingTime
         << " s\n";
  }
  if (m_totalAttrVideoDecodingTime > 0.) {
    cout << "Total attribute video sub bitstream decoding time: " << m_totalAttrVideoDecodingTime
         << " s\n";
  }
}

void MivDecoder::setGeoFrameServer(GeoFrameServer value) { m_geoFrameServer = move(value); }

void MivDecoder::setAttrFrameServer(AttrFrameServer value) { m_attrFrameServer = move(value); }

auto MivDecoder::decode() -> std::optional<AccessUnit> {
  if (haveFrame()) {
    return pullFrame();
  }
  if (decodeSequence()) {
    VERIFY_V3CBITSTREAM(haveFrame());
    return pullFrame();
  }
  return {};
}

auto MivDecoder::decodeSequence() -> bool {
  m_atlas.clear();
  m_specialAtlas.reset();

  if (auto vu = m_inputBuffer(V3cUnitHeader{VuhUnitType::V3C_VPS})) {
    m_vps = vu->v3c_payload().v3c_parameter_set();
    checkCapabilities();
    m_atlas.resize(m_vps.vps_atlas_count_minus1() + size_t(1));
  } else {
    return false;
  }

  auto vuh = V3cUnitHeader{VuhUnitType::V3C_AD};
  vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id()).vuh_atlas_id(specialAtlasId);
  while (auto vu = m_inputBuffer(vuh)) {
    decodeAsb(vu->v3c_unit_header(), vu->v3c_payload().atlas_sub_bitstream());
  }

  for (uint8_t j = 0; j <= m_vps.vps_atlas_count_minus1(); ++j) {
    auto vuh = V3cUnitHeader{VuhUnitType::V3C_AD};
    vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id())
        .vuh_atlas_id(m_vps.vps_atlas_id(j));
    while (auto vu = m_inputBuffer(vuh)) {
      decodeAsb(vu->v3c_unit_header(), vu->v3c_payload().atlas_sub_bitstream());
    }
  }

  for (uint8_t j = 0; j <= m_vps.vps_atlas_count_minus1(); ++j) {
    if (m_vps.vps_geometry_video_present_flag(j)) {
      auto vuh = V3cUnitHeader{VuhUnitType::V3C_GVD};
      vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id())
          .vuh_atlas_id(m_vps.vps_atlas_id(j));
      m_atlas[j].geoVideoServer = startVideoDecoder(vuh, m_totalGeoVideoDecodingTime);
    }
  }

  for (uint8_t j = 0; j <= m_vps.vps_atlas_count_minus1(); ++j) {
    if (m_vps.vps_attribute_video_present_flag(j)) {
      auto vuh = V3cUnitHeader{VuhUnitType::V3C_AVD};
      vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id())
          .vuh_atlas_id(m_vps.vps_atlas_id(j));
      m_atlas[j].attrVideoServer = startVideoDecoder(vuh, m_totalAttrVideoDecodingTime);
    }
  }

  return true;
}

void MivDecoder::checkCapabilities() {
  for (uint8_t j = 0; j <= m_vps.vps_atlas_count_minus1(); ++j) {
    VERIFY_MIVBITSTREAM(!m_vps.vps_auxiliary_video_present_flag(j));
    VERIFY_MIVBITSTREAM(!m_vps.vps_occupancy_video_present_flag(j));
    VERIFY_MIVBITSTREAM(m_vps.vps_geometry_video_present_flag(j));
    // TODO(BK): Add more constraints (map count, attribute count, EOM, etc.)
  }
}

auto MivDecoder::startVideoDecoder(const V3cUnitHeader &vuh, double &totalTime)
    -> std::unique_ptr<VideoDecoder::VideoServer> {
  std::string data;
  while (auto vu = m_inputBuffer(vuh)) {
    data += vu->v3c_payload().video_sub_bitstream().data();
  }
  if (data.empty()) {
    return {}; // Out-of-band?
  }

  const double t0 = clock();
  auto server = make_unique<VideoServer>(
      IVideoDecoder::create(m_vps.profile_tier_level().ptl_profile_codec_group_idc()), data);
  server->wait();
  totalTime += (clock() - t0) / CLOCKS_PER_SEC;
  return server;
}

auto MivDecoder::pullFrame() -> AccessUnit {
  assert(haveFrame());
  auto au = AccessUnit{};
  au.vps = m_vps;
  pullAtlasData(au);
  pullGeoVideoData(au);
  pullAttrVideoData(au);
  return au;
}

void MivDecoder::pullAtlasData(AccessUnit &au) {
  au.frameId = ++m_frameId;
  au.atlas.resize(m_vps.vps_atlas_count_minus1() + size_t(1));

  for (uint8_t j = 0; j <= m_vps.vps_atlas_count_minus1(); ++j) {
    assert(!m_atlas.empty() && !m_atlas[j].frames.empty());

    au.atlas[j].ath = m_atlas[j].frames.front()->ath;
    au.atlas[j].afps = afpsById(au.atlas[j].ath.ath_atlas_frame_parameter_set_id());
    au.atlas[j].asps = aspsById(au.atlas[j].afps.afps_atlas_sequence_parameter_set_id());

    au.atlas[j].viewParamsList = m_atlas[j].frames.front()->viewParamsList;
    au.atlas[j].patchParamsList = m_atlas[j].frames.front()->patchParamsList;
    au.atlas[j].blockToPatchMap = m_atlas[j].frames.front()->blockToPatchMap;

    m_atlas[j].frames.erase(m_atlas[j].frames.begin());
  }
}

auto MivDecoder::haveFrame() const -> bool {
  return !m_atlas.empty() && all_of(m_atlas.cbegin(), m_atlas.cend(),
                                    [=](const Atlas &atlas) { return !atlas.frames.empty(); });
}

void MivDecoder::pullGeoVideoData(AccessUnit &au) {
  const double t0 = clock();

  for (uint8_t j = 0; j <= m_vps.vps_atlas_count_minus1(); ++j) {
    // Get a geometry frame in-band or out-of-band
    if (m_atlas[j].geoVideoServer) {
      auto frame = m_atlas[j].geoVideoServer->getFrame();
      VERIFY_MIVBITSTREAM(frame);
      au.atlas[j].decGeoFrame = frame->as<YUV400P10>();
      m_atlas[j].geoVideoServer->wait();
    } else if (m_geoFrameServer) {
      au.atlas[j].decGeoFrame = m_geoFrameServer(j, m_frameId, au.atlas[j].decGeoFrameSize(au.vps));
    } else {
      MIVBITSTREAM_ERROR("Out-of-band geometry video data but no frame server provided");
    }
  }

  // Measure video decoding time
  const auto dt = (clock() - t0) / CLOCKS_PER_SEC;
  cout << "Time taken for decoding all geometry video sub bitstreams, one frame: " << dt << " s\n";
  m_totalGeoVideoDecodingTime += dt;
}

void MivDecoder::pullAttrVideoData(AccessUnit &au) {
  const double t0 = clock();

  for (uint8_t j = 0; j <= m_vps.vps_atlas_count_minus1(); ++j) {
    if (au.vps.vps_attribute_video_present_flag(j)) {
      // Get a texture frame in-band or out-of-band
      if (m_atlas[j].attrVideoServer) {
        auto frame = m_atlas[j].attrVideoServer->getFrame();
        VERIFY_MIVBITSTREAM(frame);
        au.atlas[j].attrFrame = frame->as<YUV444P10>();
        m_atlas[j].attrVideoServer->wait();
      } else if (m_attrFrameServer) {
        au.atlas[j].attrFrame = m_attrFrameServer(uint8_t(j), m_frameId, au.atlas[j].frameSize());
      } else {
        MIVBITSTREAM_ERROR("Out-of-band attribute video data but no frame server provided");
      }
    } else {
      // No texture: pass out a neutral gray frame
      au.atlas[j].attrFrame.resize(au.atlas[j].asps.asps_frame_width(),
                                   au.atlas[j].asps.asps_frame_height());
      au.atlas[j].attrFrame.fillNeutral();
      VERIFY_MIVBITSTREAM(au.atlas[j].decGeoFrameSize(au.vps) == au.atlas[j].frameSize());
    }
  }

  // Measure video decoding time
  const auto dt = (clock() - t0) / CLOCKS_PER_SEC;
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
  case NalUnitType::NAL_V3C_AUD:
  case NalUnitType::NAL_EOS:
  case NalUnitType::NAL_EOB:
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
  auto &x = atlasById(vuh.vuh_atlas_id());

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
    const auto &aaps = aapsById(ath.ath_atlas_adaptation_parameter_set_id());

    if (aaps.aaps_miv_extension_flag()) {
      const auto &aame = aaps.aaps_miv_extension();
      const auto &mvpl = aame.miv_view_params_list();
      frame->viewParamsList = decodeMvpl(mvpl);
    }

    const auto &afps = afpsById(ath.ath_atlas_frame_parameter_set_id());
    const auto &asps = aspsById(afps.afps_atlas_sequence_parameter_set_id());
    const auto &atdu = atl.atlas_tile_data_unit();

    frame->patchParamsList = decodeAtdu(atdu, atl.atlas_tile_header(), asps);
    frame->blockToPatchMap = decodeBlockToPatchMap(atdu, asps);

    x.frames.push_back(frame);
    x.intraFrame = frame;
  }
}

auto MivDecoder::decodeMvpl(const MivViewParamsList &mvpl) -> ViewParamsList {
  auto x = vector<ViewParams>(mvpl.mvp_num_views_minus1() + size_t(1));

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

void MivDecoder::decodeAsps(AtlasSequenceParameterSetRBSP asps) {
  for (auto &x : m_aspsV) {
    if (x.asps_atlas_sequence_parameter_set_id() == asps.asps_atlas_sequence_parameter_set_id()) {
      x = std::move(asps);
      return;
    }
  }
  m_aspsV.push_back(std::move(asps));
}

void MivDecoder::decodeAfps(AtlasFrameParameterSetRBSP afps) {
  for (auto &x : m_afpsV) {
    if (x.afps_atlas_frame_parameter_set_id() == afps.afps_atlas_frame_parameter_set_id()) {
      x = std::move(afps);
      return;
    }
  }
  m_afpsV.push_back(std::move(afps));
}

void MivDecoder::decodeAaps(AtlasAdaptationParameterSetRBSP aaps) {
  for (auto &x : m_aapsV) {
    if (x.aaps_atlas_adaptation_parameter_set_id() ==
        aaps.aaps_atlas_adaptation_parameter_set_id()) {
      x = std::move(aaps);
      return;
    }
  }
  m_aapsV.push_back(std::move(aaps));
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

void MivDecoder::parseAsps(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  VERIFY_V3CBITSTREAM(nu.nal_unit_header().nal_temporal_id_plus1() - 1 == 0);
  decodeAsps(AtlasSequenceParameterSetRBSP::decodeFrom(stream, vuh, m_vps));
}

void MivDecoder::parseAfps(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAfps(AtlasFrameParameterSetRBSP::decodeFrom(stream, m_aspsV));
}

void MivDecoder::parseAaps(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAaps(AtlasAdaptationParameterSetRBSP::decodeFrom(stream, m_vps));
}

void MivDecoder::parseAtl(const V3cUnitHeader &vuh, const NalUnit &nu) {
  istringstream stream{nu.rbsp()};
  decodeAtl(vuh, nu.nal_unit_header(),
            AtlasTileLayerRBSP::decodeFrom(stream, vuh, m_vps, m_aspsV, m_afpsV));
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

auto MivDecoder::aspsById(int id) const noexcept -> const AtlasSequenceParameterSetRBSP & {
  for (auto &x : m_aspsV) {
    if (id == x.asps_atlas_sequence_parameter_set_id()) {
      return x;
    }
  }
  V3CBITSTREAM_ERROR("Unknown ASPS ID");
}

auto MivDecoder::afpsById(int id) const noexcept -> const AtlasFrameParameterSetRBSP & {
  for (auto &x : m_afpsV) {
    if (id == x.afps_atlas_frame_parameter_set_id()) {
      return x;
    }
  }
  V3CBITSTREAM_ERROR("Unknown AFPS ID");
}

auto MivDecoder::aapsById(int id) const noexcept -> const AtlasAdaptationParameterSetRBSP & {
  for (auto &x : m_aapsV) {
    if (id == x.aaps_atlas_adaptation_parameter_set_id()) {
      return x;
    }
  }
  V3CBITSTREAM_ERROR("Unknown AAPS ID");
}

auto MivDecoder::atlasById(int id) noexcept -> Atlas & {
  for (uint8_t j = 0; j <= m_vps.vps_atlas_count_minus1(); ++j) {
    if (m_vps.vps_atlas_id(j) == id) {
      return m_atlas[j];
    }
  }
  V3CBITSTREAM_ERROR("Unknown vuh_atlas_id");
}
} // namespace TMIV::Decoder
