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
      std::cout << "WARNING: Ignoring V3C unit:\n" << *i;
      i = m_buffer.erase(i);
    } else {
      ++i;
    }
  }
}

SpecialAtlasDecoder::SpecialAtlasDecoder(V3cUnitSource source, const V3cParameterSet &vps)
    : m_source{source}, m_vps{vps} {}

auto SpecialAtlasDecoder::operator()() -> std::optional<AccessUnit> {
  auto au = std::optional<AccessUnit>{};

  if (auto asb = m_source()) {
    for (auto &x : asb->v3c_payload().atlas_sub_bitstream().nal_units()) {
      VERIFY_MIVBITSTREAM(!au && x.nal_unit_header().nal_unit_type() == NalUnitType::NAL_AAPS);
      istringstream substream{x.rbsp()};
      au = AccessUnit{};
      au->aaps = AtlasAdaptationParameterSetRBSP::decodeFrom(substream, m_vps);
    }
  }
  return au;
}

AtlasDecoder::AtlasDecoder(V3cUnitSource source, const V3cUnitHeader &vuh,
                           const V3cParameterSet &vps)
    : m_source{source}, m_vuh{vuh}, m_vps{vps} {}

auto AtlasDecoder::operator()() -> std::optional<AccessUnit> {
  if (!m_buffer.empty() || decodeAsb()) {
    return decodeAu();
  }
  return {};
}

auto AtlasDecoder::decodeAsb() -> bool {
  if (auto asb = m_source()) {
    assert(m_vuh == asb->v3c_unit_header());
    for (auto &nu : asb->v3c_payload().atlas_sub_bitstream().nal_units()) {
      if (nu.nal_unit_header().nal_layer_id() == 0) {
        m_buffer.push_back(std::move(nu));
      } else {
        std::cout << "WARNING: Ignoring NAL unit:\n" << nu;
      }
    }
    VERIFY_V3CBITSTREAM(!m_buffer.empty());
    return true;
  }
  return false;
}

namespace {
constexpr bool isAud(NalUnitType nut) noexcept {
  return nut == NalUnitType::NAL_AUD || nut == NalUnitType::NAL_V3C_AUD;
}

constexpr bool isPrefixNalUnit(NalUnitType nut) noexcept {
  return nut == NalUnitType::NAL_ASPS || nut == NalUnitType::NAL_AFPS ||
         nut == NalUnitType::NAL_PREFIX_NSEI || nut == NalUnitType::NAL_PREFIX_ESEI ||
         (NalUnitType::NAL_AAPS <= nut && nut <= NalUnitType::NAL_RSV_NACL_50) ||
         (NalUnitType::NAL_UNSPEC_53 <= nut && nut <= NalUnitType::NAL_UNSPEC_57);
}

constexpr bool isAcl(NalUnitType nut) noexcept { return nut <= NalUnitType::NAL_RSV_ACL_35; }

constexpr bool isSuffixNalUnit(NalUnitType nut) noexcept {
  return nut == NalUnitType::NAL_FD || nut == NalUnitType::NAL_SUFFIX_NSEI ||
         nut == NalUnitType::NAL_SUFFIX_ESEI || nut == NalUnitType::NAL_RSV_NACL_51 ||
         nut == NalUnitType::NAL_RSV_NACL_52 || NalUnitType::NAL_UNSPEC_58 <= nut;
}

constexpr bool isEos(NalUnitType nut) noexcept { return nut == NalUnitType::NAL_EOS; }

constexpr bool isEob(NalUnitType nut) noexcept { return nut == NalUnitType::NAL_EOB; }
} // namespace

auto AtlasDecoder::decodeAu() -> AccessUnit {
  auto au = AccessUnit{};
  au.foc = ++m_foc;

  const auto nut = [this]() { return m_buffer.front().nal_unit_header().nal_unit_type(); };

  if (!m_buffer.empty() && isAud(nut())) {
    m_buffer.pop_front();
  }

  while (!m_buffer.empty() && isPrefixNalUnit(nut())) {
    decodePrefixNalUnit(au, m_buffer.front());
    m_buffer.pop_front();
  }

  VERIFY_V3CBITSTREAM(!m_buffer.empty() && isAcl(nut()));
  decodeAclNalUnit(au, m_buffer.front());
  m_buffer.pop_front();

  while (!m_buffer.empty() && isSuffixNalUnit(nut())) {
    decodeSuffixNalUnit(au, m_buffer.front());
    m_buffer.pop_front();
  }

  if (!m_buffer.empty() && isEos(nut())) {
    m_buffer.pop_front();
  }

  if (!m_buffer.empty() && isEob(nut())) {
    m_buffer.pop_front();
  }

  // TODO(BK): Set FOC

  return au;
}

void AtlasDecoder::decodePrefixNalUnit(AccessUnit &au, const NalUnit &nu) {
  const auto &nuh = nu.nal_unit_header();
  if (nuh.nal_layer_id() != 0) {
    std::cout << " WARNING: Ignoring NAL unit:\n" << nuh;
    return;
  }

  istringstream stream{nu.rbsp()};

  switch (nuh.nal_unit_type()) {
  case NalUnitType::NAL_ASPS:
    return decodeAsps(stream);
  case NalUnitType::NAL_AFPS:
    return decodeAfps(stream);
  case NalUnitType::NAL_AAPS:
    return decodeAaps(stream);
  case NalUnitType::NAL_PREFIX_ESEI:
    return decodeSei(au.prefixESei, stream);
  case NalUnitType::NAL_PREFIX_NSEI:
    return decodeSei(au.prefixNSei, stream);
  default:
    std::cout << "WARNING: Ignoring NAL unit:\n" << nu;
  }
}

void AtlasDecoder::decodeAclNalUnit(AccessUnit &au, const NalUnit &nu) {
  const auto &nuh = nu.nal_unit_header();
  if (nuh.nal_layer_id() != 0) {
    std::cout << " WARNING: Ignoring NAL unit:\n" << nu;
    return;
  }

  istringstream stream{nu.rbsp()};

  au.atl = AtlasTileLayerRBSP::decodeFrom(stream, m_vuh, m_vps, m_aspsV, m_afpsV);
  au.afps = afpsById(m_afpsV, au.atl.atlas_tile_header().ath_atlas_frame_parameter_set_id());
  au.asps = aspsById(m_aspsV, au.afps.afps_atlas_sequence_parameter_set_id());
}

void AtlasDecoder::decodeSuffixNalUnit(AccessUnit &au, const NalUnit &nu) {
  const auto &nuh = nu.nal_unit_header();
  if (nuh.nal_layer_id() != 0) {
    std::cout << " WARNING: Ignoring NAL unit:\n" << nu;
    return;
  }

  istringstream stream{nu.rbsp()};

  switch (nuh.nal_unit_type()) {
  case NalUnitType::NAL_FD:
    return;
  case NalUnitType::NAL_SUFFIX_ESEI:
    return decodeSei(au.suffixESei, stream);
  case NalUnitType::NAL_SUFFIX_NSEI:
    return decodeSei(au.suffixNSei, stream);
  default:
    std::cout << "WARNING: Ignoring NAL unit:\n" << nu;
  }
}

void AtlasDecoder::decodeAsps(std::istream &stream) {
  auto asps = AtlasSequenceParameterSetRBSP::decodeFrom(stream, m_vuh, m_vps);
  for (auto &x : m_aspsV) {
    if (x.asps_atlas_sequence_parameter_set_id() == asps.asps_atlas_sequence_parameter_set_id()) {
      x = std::move(asps);
      return;
    }
  }
  return m_aspsV.push_back(asps);
}

void AtlasDecoder::decodeAfps(std::istream &stream) {
  auto afps = AtlasFrameParameterSetRBSP::decodeFrom(stream, m_aspsV);
  for (auto &x : m_afpsV) {
    if (x.afps_atlas_frame_parameter_set_id() == afps.afps_atlas_frame_parameter_set_id()) {
      x = std::move(afps);
      return;
    }
  }
  return m_afpsV.push_back(afps);
}

void AtlasDecoder::decodeAaps(std::istream &stream) {
  auto aaps = AtlasAdaptationParameterSetRBSP::decodeFrom(stream, m_vps);
  for (auto &x : m_aapsV) {
    if (x.aaps_atlas_adaptation_parameter_set_id() ==
        aaps.aaps_atlas_adaptation_parameter_set_id()) {
      x = std::move(aaps);
      return;
    }
  }
  return m_aapsV.push_back(aaps);
}

void AtlasDecoder::decodeSei(std::vector<SeiMessage> &messages, std::istream &stream) {
  auto sei = SeiRBSP::decodeFrom(stream);
  for (auto &x : sei.messages()) {
    messages.push_back(x);
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

auto MivDecoder::operator()() -> std::optional<AccessUnit> {
  ++m_au.foc;
  m_au.irap = expectIrap();

  if (m_au.irap && !decodeVps()) {
    return {};
  }

  if (!m_specialAtlasAu) {
    m_specialAtlasAu = (*m_specialAtlasDecoder)();
    VERIFY_MIVBITSTREAM(m_specialAtlasAu.has_value());
    decodeSpecialAtlas();
  }
  for (uint8_t j = 0; j <= m_au.vps.vps_atlas_count_minus1(); ++j) {
    if (!m_atlasAu[j] || m_atlasAu[j]->foc < m_au.foc) {
      m_atlasAu[j] = (*m_atlasDecoder[j])();
      if (m_atlasAu[j] && m_atlasAu[j]->foc == m_au.foc) {
        decodeAtlas(j);
      }
    }
  }

  auto result = std::array{false, false};

  for (uint8_t j = 0; j <= m_au.vps.vps_atlas_count_minus1(); ++j) {
    if (m_au.vps.vps_geometry_video_present_flag(j)) {
      result[decodeGeoVideo(j)] = true;
    }
  }
  for (uint8_t j = 0; j <= m_au.vps.vps_atlas_count_minus1(); ++j) {
    if (m_au.vps.vps_attribute_video_present_flag(j)) {
      result[decodeAttrVideo(j)] = true;
    }
  }

  if (result[false] && result[true]) {
    throw runtime_error("One of the video streams is truncated");
  }
  if (result[true]) {
    // TODO(BK): This copies the video frames.
    return m_au;
  }
  return {};
}

auto MivDecoder::expectIrap() const -> bool { return !m_specialAtlasDecoder; }

auto MivDecoder::decodeVps() -> bool {
  auto vu = m_inputBuffer(V3cUnitHeader{VuhUnitType::V3C_VPS});
  if (!vu) {
    return false;
  }
  m_au.vps = vu->v3c_payload().v3c_parameter_set();

  checkCapabilities();

  auto vuh = V3cUnitHeader{VuhUnitType::V3C_AD};
  vuh.vuh_atlas_id(specialAtlasId);
  m_specialAtlasDecoder =
      make_unique<SpecialAtlasDecoder>([this, vuh]() { return m_inputBuffer(vuh); }, m_au.vps);

  m_atlasDecoder.clear();
  m_atlasAu.assign(m_au.vps.vps_atlas_count_minus1() + size_t(1), {});
  m_au.atlas.clear();
  m_geoVideoDecoder.clear();
  m_attrVideoDecoder.clear();

  for (uint8_t j = 0; j <= m_au.vps.vps_atlas_count_minus1(); ++j) {
    auto vuh = V3cUnitHeader{VuhUnitType::V3C_AD};
    vuh.vuh_atlas_id(m_au.vps.vps_atlas_id(j));
    m_atlasDecoder.push_back(
        make_unique<AtlasDecoder>([this, vuh]() { return m_inputBuffer(vuh); }, vuh, m_au.vps));
    m_au.atlas.emplace_back();

    if (m_au.vps.vps_geometry_video_present_flag(j)) {
      auto vuh = V3cUnitHeader{VuhUnitType::V3C_GVD};
      vuh.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id())
          .vuh_atlas_id(m_au.vps.vps_atlas_id(j));
      m_geoVideoDecoder.push_back(startVideoDecoder(vuh, m_totalGeoVideoDecodingTime));
    }

    if (m_au.vps.vps_attribute_video_present_flag(j)) {
      auto vuh = V3cUnitHeader{VuhUnitType::V3C_AVD};
      vuh.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id())
          .vuh_atlas_id(m_au.vps.vps_atlas_id(j));
      m_attrVideoDecoder.push_back(startVideoDecoder(vuh, m_totalGeoVideoDecodingTime));
    }
  }

  return true;
}

void MivDecoder::checkCapabilities() {
  VERIFY_MIVBITSTREAM(m_au.vps.vps_miv_extension_flag());

  for (uint8_t j = 0; j <= m_au.vps.vps_atlas_count_minus1(); ++j) {
    VERIFY_MIVBITSTREAM(!m_au.vps.vps_auxiliary_video_present_flag(j));
    VERIFY_MIVBITSTREAM(!m_au.vps.vps_occupancy_video_present_flag(j));
    VERIFY_MIVBITSTREAM(m_au.vps.vps_geometry_video_present_flag(j));
    // TODO(BK): Add more constraints (map count, attribute count, EOM, etc.)
  }
}

auto MivDecoder::startVideoDecoder(const V3cUnitHeader &vuh, double &totalTime)
    -> std::unique_ptr<VideoDecoder::VideoServer> {
  std::string data;
  while (auto vu = m_inputBuffer(vuh)) {
    // TODO(BK): Let the video decoder pull V3C units. This implementation assumes the bitstream is
    // short enough to fit in memory. The reason for this shortcut is that the change requires
    // parsing of the Annex B byte stream, which can be easily done but it requires an additional
    // implementation effort.
    data += vu->v3c_payload().video_sub_bitstream().data();
  }
  if (data.empty()) {
    return {}; // Out-of-band?
  }

  const double t0 = clock();
  auto server = make_unique<VideoServer>(
      IVideoDecoder::create(m_au.vps.profile_tier_level().ptl_profile_codec_group_idc()), data);
  server->wait();
  totalTime += (clock() - t0) / CLOCKS_PER_SEC;
  return server;
}

void MivDecoder::decodeSpecialAtlas() {
  // TODO(BK): Implement after switch to common atlas frame
  decodeViewParamsList();
}

void MivDecoder::decodeViewParamsList() {
  // TODO(BK): Implement camera parameter updates

  const auto &mvpl = m_specialAtlasAu->aaps.aaps_miv_extension().miv_view_params_list();
  m_au.viewParamsList.assign(mvpl.mvp_num_views_minus1() + size_t(1), {});

  for (uint16_t viewId = 0; viewId <= mvpl.mvp_num_views_minus1(); ++viewId) {
    m_au.viewParamsList[viewId].ce = mvpl.camera_extrinsics(viewId);
    m_au.viewParamsList[viewId].ci = mvpl.camera_intrinsics(viewId);
    m_au.viewParamsList[viewId].dq = mvpl.depth_quantization(viewId);

    if (mvpl.mvp_pruning_graph_params_present_flag()) {
      m_au.viewParamsList[viewId].pp = mvpl.pruning_parent(viewId);
    }
  }
}

void MivDecoder::decodeAtlas(uint8_t j) {
  m_au.atlas[j].asps = m_atlasAu[j]->asps;
  m_au.atlas[j].afps = m_atlasAu[j]->afps;
  decodeBlockToPatchMap(j);
  decodePatchParamsList(j);
}

void MivDecoder::decodeBlockToPatchMap(uint8_t j) {
  auto &btpm = m_au.atlas[j].blockToPatchMap;
  const auto &asps = m_au.atlas[j].asps;
  btpm = BlockToPatchMap{asps.asps_frame_width() >> asps.asps_log2_patch_packing_block_size(),
                         asps.asps_frame_height() >> asps.asps_log2_patch_packing_block_size()};
  fill(btpm.getPlane(0).begin(), btpm.getPlane(0).end(), unusedPatchId);

  m_atlasAu[j]->atl.atlas_tile_data_unit().visit(
      [&btpm](size_t p, AtduPatchMode /* unused */, const PatchInformationData &pid) {
        const auto &pdu = pid.patch_data_unit();
        const auto first = Vec2i{pdu.pdu_2d_pos_x(), pdu.pdu_2d_pos_y()};
        const auto last = first + Vec2i{pdu.pdu_2d_size_x_minus1(), pdu.pdu_2d_size_y_minus1()};

        for (int y = first.y(); y <= last.y(); ++y) {
          for (int x = first.x(); x <= last.x(); ++x) {
            btpm.getPlane(0)(y, x) = uint16_t(p);
          }
        }
      });
}

void MivDecoder::decodePatchParamsList(uint8_t j) {
  const auto &ath = m_atlasAu[j]->atl.atlas_tile_header();
  VERIFY_MIVBITSTREAM(ath.ath_type() == AthType::I_TILE || ath.ath_type() == AthType::SKIP_TILE);
  if (ath.ath_type() == AthType::SKIP_TILE) {
    return;
  }

  const auto &atdu = m_atlasAu[j]->atl.atlas_tile_data_unit();
  const auto &asps = m_atlasAu[j]->asps;
  auto &ppl = m_au.atlas[j].patchParamsList;
  ppl.assign(atdu.atduTotalNumberOfPatches(), {});

  atdu.visit([&](size_t p, AtduPatchMode /* unused */, const PatchInformationData &pid) {
    const auto &pdu = pid.patch_data_unit();
    const auto k = asps.asps_log2_patch_packing_block_size();

    ppl[p].pduOrientationIndex(pdu.pdu_orientation_index());
    ppl[p].pdu2dPos({int(pdu.pdu_2d_pos_x() << k), int(pdu.pdu_2d_pos_y() << k)});
    ppl[p].pdu2dSize(
        {int((pdu.pdu_2d_size_x_minus1() + 1U) << k), int((pdu.pdu_2d_size_y_minus1() + 1U) << k)});
    ppl[p].pduViewPos({pdu.pdu_view_pos_x(), pdu.pdu_view_pos_y()});
    ppl[p].pduDepthStart(pdu.pdu_depth_start() << ath.ath_pos_min_z_quantizer());
    ppl[p].pduViewId(pdu.pdu_projection_id());

    if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
      ppl[p].pduDepthEnd(pdu.pdu_depth_end() << ath.ath_pos_delta_max_z_quantizer());
    }
    if (asps.asps_miv_extension_flag()) {
      ppl[p].pduEntityId(pdu.pdu_miv_extension().pdu_entity_id());

      if (asps.asps_miv_extension().asme_depth_occ_threshold_flag()) {
        ppl[p].pduDepthOccMapThreshold(pdu.pdu_miv_extension().pdu_depth_occ_threshold());
      }
    }
  });
}

auto MivDecoder::decodeGeoVideo(uint8_t j) -> bool {
  const double t0 = clock();

  if (m_geoVideoDecoder[j]) {
    auto frame = m_geoVideoDecoder[j]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[j].decGeoFrame = frame->as<YUV400P10>();
    m_geoVideoDecoder[j]->wait();
  } else if (m_geoFrameServer) {
    m_au.atlas[j].decGeoFrame = m_geoFrameServer(m_au.vps.vps_atlas_id(j), m_au.foc,
                                                 m_au.atlas[j].decGeoFrameSize(m_au.vps));
    if (m_au.atlas[j].decGeoFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band geometry video data but no frame server provided");
  }

  m_totalGeoVideoDecodingTime += (clock() - t0) / CLOCKS_PER_SEC;
  return true;
}

auto MivDecoder::decodeAttrVideo(uint8_t j) -> bool {
  const double t0 = clock();

  if (m_attrVideoDecoder[j]) {
    auto frame = m_attrVideoDecoder[j]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[j].attrFrame = frame->as<YUV444P10>();
    m_attrVideoDecoder[j]->wait();
  } else if (m_attrFrameServer) {
    m_au.atlas[j].attrFrame =
        m_attrFrameServer(m_au.vps.vps_atlas_id(j), m_au.foc, m_au.atlas[j].frameSize());
    if (m_au.atlas[j].attrFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band geometry video data but no frame server provided");
  }

  m_totalGeoVideoDecodingTime += (clock() - t0) / CLOCKS_PER_SEC;
  return true;
}
} // namespace TMIV::Decoder
