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

#include <TMIV/Encoder/EncodeMiv.h>

#include <TMIV/Common/FlatMap.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/SeiRBSP.h>

namespace TMIV::Encoder {
namespace {
using MivBitstream::V3cUnit;
using VUH = MivBitstream::V3cUnitHeader;
using NUT = MivBitstream::NalUnitType;

const auto nuhAsps = MivBitstream::NalUnitHeader{NUT::NAL_ASPS, 0, 1};
const auto nuhAfps = MivBitstream::NalUnitHeader{NUT::NAL_AFPS, 0, 1};
const auto nuhIdr = MivBitstream::NalUnitHeader{NUT::NAL_IDR_N_LP, 0, 1};
const auto nuhTrail = MivBitstream::NalUnitHeader{NUT::NAL_TRAIL_N, 0, 1};

const auto nuhCasps = MivBitstream::NalUnitHeader{NUT::NAL_CASPS, 0, 1};
const auto nuhCafIdr = MivBitstream::NalUnitHeader{NUT::NAL_CAF_IDR, 0, 1};
const auto nuhCafTrial = MivBitstream::NalUnitHeader{NUT::NAL_CAF_TRIAL, 0, 1};

const auto nuhPrefixNsei = MivBitstream::NalUnitHeader{NUT::NAL_PREFIX_NSEI, 0, 1};
const auto nuhSuffixNsei = MivBitstream::NalUnitHeader{NUT::NAL_SUFFIX_NSEI, 0, 1};

class MivEncoder {
public:
  MivEncoder(std::function<void(std::optional<V3cUnit>)> sink, bool rewriteParameterSets)
      : m_sink{std::move(sink)}, m_rewriteParameterSets{rewriteParameterSets} {}

  void operator()(std::optional<EncoderParams> frame) {
    VERIFY(m_sink);

    if (frame) {
      encode(*frame);
    } else {
      flush();
      m_sink(std::nullopt);
      m_sink = nullptr;
    }
  }

private:
  struct PreviouslySentMessages {
    MivBitstream::ViewParamsList viewParamsList{};
    std::optional<MivBitstream::ViewingSpace> viewingSpace{};
    std::optional<MivBitstream::ViewportCameraParameters> viewportCameraParameters{};
    std::optional<MivBitstream::ViewportPosition> viewportPosition{};
  };

  struct AtlasSubBitstream : public MivBitstream::AtlasSubBitstream {
  public:
    AtlasSubBitstream() : MivBitstream::AtlasSubBitstream{MivBitstream::SampleStreamNalHeader{2}} {}
  };

  [[nodiscard]] constexpr auto maxFrmOrderCntLsb() const {
    return 1U << (m_log2MaxFrmOrderCntLsbMinus4 + 4U);
  }

  enum class State { initial, irap, nonIrap, ready };

  void encode(EncoderParams params) {
    m_params = std::move(params);

    checkBitstreamRequirements();

    switch (m_state) {
    case State::initial:
      VERIFY(m_params.foc == 0);
      break;
    case State::ready:
      m_state = m_params.foc == 0 ? State::irap : State::nonIrap;
      break;
    default:
      UNREACHABLE;
    }

    if (m_state == State::nonIrap) {
      m_frmOrderCntLsb = Common::downCast<uint16_t>(m_params.foc % maxFrmOrderCntLsb());
      VERIFY_MIVBITSTREAM(m_frmOrderCntLsb < maxFrmOrderCntLsb());
    } else {
      m_frmOrderCntLsb = 0;
    }

    if (writeParameterSets()) {
      flush();
      m_params.vps.profile_tier_level().ptl_max_decodes_idc(ptlMaxDecodesIdc());
      m_params.vps.calculateExtensionLengths();
      m_sink(V3cUnit{VUH::vps(), m_params.vps});
      m_log2MaxFrmOrderCntLsbMinus4 =
          m_params.casps.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4();
    }

    // NOTE(#253): always write even for non-IRAP intra periods w/o view parameter updates
    //             to avoid frame order count ambiguity
    commonAtlasSubBitstream();
    m_previouslySentMessages.viewParamsList = m_params.viewParamsList;

    for (uint8_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
      // Clause 7.4.5.3.2 of V-PCC DIS d85 [N19329]: AXPS regardless of atlas ID (and temporal ID)
      // share the same value space for AXPS ID
      auto &aau = m_params.atlas[k];
      aau.asps.asps_atlas_sequence_parameter_set_id(k);
      aau.afps.afps_atlas_frame_parameter_set_id(k);
      aau.afps.afps_atlas_sequence_parameter_set_id(k);

      aau.athTemplate.ath_atlas_frame_parameter_set_id(k).ath_atlas_frm_order_cnt_lsb(
          m_frmOrderCntLsb);

      atlasSubBitstream(k);
    }

    m_state = State::ready;
  }

  auto flush() -> bool {
    auto flushed = false;

    for (auto &[vuh, asb] : m_asbBuffers) {
      flushed = flushed || !asb.nal_units().empty();
      m_sink(V3cUnit{vuh, std::move(asb)});
    }

    m_asbBuffers.clear();
    return flushed;
  }

  [[nodiscard]] auto writeParameterSets() const -> bool {
    switch (m_state) {
    case State::initial:
      return true;
    case State::irap:
      return m_rewriteParameterSets;
    case State::nonIrap:
      return false;
    case State::ready:
      UNREACHABLE;
    }
    UNREACHABLE;
  }

  [[nodiscard]] auto ptlMaxDecodesIdc() const -> MivBitstream::PtlMaxDecodesIdc {
    auto numDecodes = 0;
    for (uint8_t k = 0; k < m_params.vps.vps_atlas_count_minus1() + 1; ++k) {
      const auto j = m_params.vps.vps_atlas_id(k);
      numDecodes += static_cast<int32_t>(m_params.vps.vps_auxiliary_video_present_flag(j));
      numDecodes += static_cast<int32_t>(m_params.vps.vps_occupancy_video_present_flag(j));
      numDecodes += static_cast<int32_t>(m_params.vps.vps_geometry_video_present_flag(j)) *
                    (m_params.vps.vps_map_count_minus1(j) + 1);
      if (m_params.vps.vps_attribute_video_present_flag(j)) {
        numDecodes += m_params.vps.attribute_information(j).ai_attribute_count() *
                      (m_params.vps.vps_map_count_minus1(j) + 1);
      }
    }
    if (numDecodes <= 1) {
      return MivBitstream::PtlMaxDecodesIdc::max_1;
    }
    if (numDecodes <= 2) {
      return MivBitstream::PtlMaxDecodesIdc::max_2;
    }
    if (numDecodes <= 3) {
      return MivBitstream::PtlMaxDecodesIdc::max_3;
    }
    if (numDecodes <= 4) {
      return MivBitstream::PtlMaxDecodesIdc::max_4;
    }
    if (numDecodes <= 6) {
      return MivBitstream::PtlMaxDecodesIdc::max_6;
    }
    if (numDecodes <= 12) {
      return MivBitstream::PtlMaxDecodesIdc::max_12;
    }
    if (numDecodes <= 16) {
      return MivBitstream::PtlMaxDecodesIdc::max_16;
    }
    if (numDecodes <= 24) {
      return MivBitstream::PtlMaxDecodesIdc::max_24;
    }
    if (numDecodes <= 24) {
      return MivBitstream::PtlMaxDecodesIdc::max_24;
    }
    if (numDecodes <= 32) {
      return MivBitstream::PtlMaxDecodesIdc::max_32;
    }
    return MivBitstream::PtlMaxDecodesIdc::unconstrained;
  }

  static void encodeSeiRbspToAsb(MivBitstream::AtlasSubBitstream &asb,
                                 const MivBitstream::SeiRBSP &seiRbsp,
                                 const MivBitstream::NalUnitHeader &nuh) {
    std::ostringstream subStream;
    seiRbsp.encodeTo(subStream, nuh.nal_unit_type());
    asb.nal_units().emplace_back(nuh, subStream.str());
  }

  void commonAtlasSubBitstream() {
    auto &asb = m_asbBuffers[VUH::cad(m_params.vps.vps_v3c_parameter_set_id())];

    encodePrefixSeiMessages(asb);

    if (writeParameterSets()) {
      writeNalUnit(asb, nuhCasps, m_params.casps);
    }
    if (m_state == State::nonIrap) {
      writeNalUnit(asb, nuhCafTrial, commonAtlasFrame(), nuhCafTrial, std::vector{m_params.casps},
                   maxFrmOrderCntLsb());
    } else {
      writeNalUnit(asb, nuhCafIdr, idrCommonAtlasFrame(), nuhCafIdr, std::vector{m_params.casps},
                   maxFrmOrderCntLsb());
    }
    encodeSuffixSeiMessages(asb);
  }

  [[nodiscard]] auto idrCommonAtlasFrame() const -> MivBitstream::CommonAtlasFrameRBSP {
    auto caf = MivBitstream::CommonAtlasFrameRBSP{}
                   .caf_common_atlas_sequence_parameter_set_id(0)
                   .caf_common_atlas_frm_order_cnt_lsb(m_frmOrderCntLsb);
    caf.caf_miv_extension().miv_view_params_list() = mivViewParamsList();
    return caf;
  }

  [[nodiscard]] auto commonAtlasFrame() const -> MivBitstream::CommonAtlasFrameRBSP {
    auto caf = MivBitstream::CommonAtlasFrameRBSP{}
                   .caf_common_atlas_sequence_parameter_set_id(0)
                   .caf_common_atlas_frm_order_cnt_lsb(m_frmOrderCntLsb);
    auto &came = caf.caf_miv_extension();

    const auto &viewParamsList = m_previouslySentMessages.viewParamsList;
    VERIFY_MIVBITSTREAM(viewParamsList.size() == m_params.viewParamsList.size());
    came.came_update_extrinsics_flag(false);
    came.came_update_intrinsics_flag(false);
    came.came_update_depth_quantization_flag(false);

    for (size_t i = 0; i < viewParamsList.size(); ++i) {
      if (!viewParamsList[i].pose.hasEqualCodeTo(m_params.viewParamsList[i].pose)) {
        came.came_update_extrinsics_flag(true);
      }
      if (viewParamsList[i].ci != m_params.viewParamsList[i].ci) {
        came.came_update_intrinsics_flag(true);
      }
      if (viewParamsList[i].dq != m_params.viewParamsList[i].dq) {
        came.came_update_depth_quantization_flag(true);
      }
    }
    if (came.came_update_extrinsics_flag()) {
      came.miv_view_params_update_extrinsics() = mivViewParamsUpdateExtrinsics();
    }
    if (came.came_update_intrinsics_flag()) {
      came.miv_view_params_update_intrinsics() = mivViewParamsUpdateIntrinsics();
    }
    if (came.came_update_depth_quantization_flag()) {
      came.miv_view_params_update_depth_quantization() = mivViewParamsUpdateDepthQuantization();
    }

    return caf;
  }

  [[nodiscard]] auto mivViewParamsList() const -> MivBitstream::MivViewParamsList {
    auto mvpl = MivBitstream::MivViewParamsList{};
    const auto &vpl = m_params.viewParamsList;

    PRECONDITION(!vpl.empty());
    mvpl.mvp_num_views_minus1(static_cast<uint16_t>(vpl.size() - 1));
    mvpl.mvp_intrinsic_params_equal_flag(
        std::all_of(vpl.begin(), vpl.end(), [&](const auto &x) { return x.ci == vpl.front().ci; }));
    mvpl.mvp_depth_quantization_params_equal_flag(
        std::all_of(vpl.begin(), vpl.end(), [&](const auto &x) { return x.dq == vpl.front().dq; }));
    mvpl.mvp_pruning_graph_params_present_flag(vpl.front().pp.has_value());

    for (uint16_t i = 0; i <= mvpl.mvp_num_views_minus1(); ++i) {
      const auto &vp = vpl[i];
      mvpl.camera_extrinsics(i) = vp.pose.encodeToCameraExtrinsics();
      mvpl.mvp_inpaint_flag(i, vp.viewInpaintFlag);
      mvpl.mvp_view_background_flag(i, vp.backgroundViewFlag);

      if (i == 0 || !mvpl.mvp_intrinsic_params_equal_flag()) {
        mvpl.camera_intrinsics(i) = vp.ci;
      }
      if (i == 0 || !mvpl.mvp_depth_quantization_params_equal_flag()) {
        mvpl.depth_quantization(i) = vp.dq;
      }
      PRECONDITION(vp.pp.has_value() == mvpl.mvp_pruning_graph_params_present_flag());
      if (vp.pp.has_value()) {
        mvpl.pruning_parent(i) = *vp.pp;
      }
      mvpl.chroma_scaling(i) = vp.cs;
    }

    mvpl.mvp_num_views_minus1(static_cast<uint16_t>(m_params.viewParamsList.size() - 1));

    auto consecutiveViewIds = true;

    for (size_t v = 0; v < vpl.size(); ++v) {
      if (vpl[v].viewId != MivBitstream::ViewId{v}) {
        consecutiveViewIds = false;
      }
    }

    if (!consecutiveViewIds) {
      for (size_t v = 0; v < vpl.size(); ++v) {
        mvpl.mvp_view_id(Common::assertDownCast<uint16_t>(v), vpl[v].viewId);
      }
    }

    return mvpl;
  }

  [[nodiscard]] auto mivViewParamsUpdateExtrinsics() const
      -> MivBitstream::MivViewParamsUpdateExtrinsics {
    auto mvpue = MivBitstream::MivViewParamsUpdateExtrinsics{};
    auto viewIdx = std::vector<uint16_t>{};
    for (size_t v = 0; v < m_previouslySentMessages.viewParamsList.size(); ++v) {
      if (!m_previouslySentMessages.viewParamsList[v].pose.hasEqualCodeTo(
              m_params.viewParamsList[v].pose)) {
        viewIdx.push_back(static_cast<uint16_t>(v));
      }
    }
    VERIFY_MIVBITSTREAM(!viewIdx.empty());
    mvpue.mvpue_num_view_updates_minus1(static_cast<uint16_t>(viewIdx.size() - 1));
    for (uint16_t i = 0; i <= mvpue.mvpue_num_view_updates_minus1(); ++i) {
      mvpue.mvpue_view_idx(i, viewIdx[i]);
      mvpue.camera_extrinsics(i) =
          m_params.viewParamsList[viewIdx[i]].pose.encodeToCameraExtrinsics();
    }
    return mvpue;
  }

  [[nodiscard]] auto mivViewParamsUpdateIntrinsics() const
      -> MivBitstream::MivViewParamsUpdateIntrinsics {
    auto mvpui = MivBitstream::MivViewParamsUpdateIntrinsics{};
    auto viewIdx = std::vector<uint16_t>{};
    for (size_t v = 0; v < m_previouslySentMessages.viewParamsList.size(); ++v) {
      if (m_previouslySentMessages.viewParamsList[v].ci != m_params.viewParamsList[v].ci) {
        viewIdx.push_back(static_cast<uint16_t>(v));
      }
    }
    VERIFY_MIVBITSTREAM(!viewIdx.empty());
    mvpui.mvpui_num_view_updates_minus1(static_cast<uint16_t>(viewIdx.size() - 1));
    for (uint16_t i = 0; i <= mvpui.mvpui_num_view_updates_minus1(); ++i) {
      mvpui.mvpui_view_idx(i, viewIdx[i]);
      mvpui.camera_intrinsics(i) = m_params.viewParamsList[viewIdx[i]].ci;
    }
    return mvpui;
  }

  [[nodiscard]] auto mivViewParamsUpdateDepthQuantization() const
      -> MivBitstream::MivViewParamsUpdateDepthQuantization {
    auto mvpudq = MivBitstream::MivViewParamsUpdateDepthQuantization{};
    auto viewIdx = std::vector<uint16_t>{};
    for (size_t v = 0; v < m_previouslySentMessages.viewParamsList.size(); ++v) {
      if (m_previouslySentMessages.viewParamsList[v].dq != m_params.viewParamsList[v].dq) {
        viewIdx.push_back(static_cast<uint16_t>(v));
      }
    }
    VERIFY_MIVBITSTREAM(!viewIdx.empty());
    mvpudq.mvpudq_num_view_updates_minus1(static_cast<uint16_t>(viewIdx.size() - 1));
    for (uint16_t i = 0; i <= mvpudq.mvpudq_num_view_updates_minus1(); ++i) {
      mvpudq.mvpudq_view_idx(i, viewIdx[i]);
      mvpudq.depth_quantization(i) = m_params.viewParamsList[viewIdx[i]].dq;
    }
    return mvpudq;
  }

  void atlasSubBitstream(uint8_t atlasIdx) {
    const auto vuh =
        VUH::ad(m_params.vps.vps_v3c_parameter_set_id(), m_params.vps.vps_atlas_id(atlasIdx));
    auto &asb = m_asbBuffers[vuh];
    const auto &aau = m_params.atlas[atlasIdx];

    if (writeParameterSets()) {
      VERIFY_MIVBITSTREAM(m_log2MaxFrmOrderCntLsbMinus4 ==
                          aau.asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4());
      writeNalUnit(asb, nuhAsps, aau.asps);
      writeNalUnit(asb, nuhAfps, aau.afps,
                   std::vector<MivBitstream::AtlasSequenceParameterSetRBSP>{aau.asps});
    }

    const auto aspsV = std::vector<MivBitstream::AtlasSequenceParameterSetRBSP>{aau.asps};
    const auto afpsV = std::vector<MivBitstream::AtlasFrameParameterSetRBSP>{aau.afps};
    const auto nuh = m_state == State::nonIrap ? nuhTrail : nuhIdr;

    for (size_t tileIdx = 0; tileIdx < aau.tilePartitions.size(); ++tileIdx) {
      writeNalUnit(asb, nuh, atlasTileLayer(atlasIdx, tileIdx), nuh, aspsV, afpsV);
    }
  }

  [[nodiscard]] auto atlasTileLayer(uint8_t atlasIdx, size_t tileIdx) const
      -> MivBitstream::AtlasTileLayerRBSP {
    const auto &aau = m_params.atlas[atlasIdx];
    const auto atlasId = m_params.vps.vps_atlas_id(atlasIdx);
    const auto tilePartition = aau.tilePartitions[tileIdx];

    auto x = MivBitstream::AtlasTileLayerRBSP{};

    auto &ath = x.atlas_tile_header();
    ath = m_params.atlas[atlasIdx].athTemplate;
    ath.ath_id(Common::downCast<uint8_t>(tileIdx));

    auto &atdu = x.atlas_tile_data_unit();
    auto p = size_t{};

    for (const auto &pp : m_params.patchParamsList) {
      if (pp.atlasId() == atlasId && isWithin(pp, tilePartition)) {
        atdu.atdu_patch_mode(p, MivBitstream::AtduPatchMode::I_INTRA);
        atdu.patch_information_data(p).patch_data_unit() =
            pp.encodePdu(m_params.vps, atlasId, aau.asps, aau.afps, ath, tilePartition);
        ++p;
      }
    }

    atdu.atdu_patch_mode(p, MivBitstream::AtduPatchMode::I_END);

    return x;
  }

  template <typename Payload, typename... Args>
  void writeNalUnit(MivBitstream::AtlasSubBitstream &asb, MivBitstream::NalUnitHeader nuh,
                    Payload &&payload, Args &&...args) {
    std::ostringstream substream1;
    std::forward<Payload>(payload).encodeTo(substream1, std::forward<Args>(args)...);
    asb.nal_units().emplace_back(nuh, substream1.str());
  }

  void encodePrefixSeiMessages(MivBitstream::AtlasSubBitstream &asb) {
    auto sei = MivBitstream::SeiRBSP{};

    if (m_params.viewingSpace && m_previouslySentMessages.viewingSpace != m_params.viewingSpace) {
      sei.messages().emplace_back(MivBitstream::PayloadType::viewing_space,
                                  MivBitstream::SeiPayload{*m_params.viewingSpace});
      m_previouslySentMessages.viewingSpace = m_params.viewingSpace;
    }

    if (m_params.viewportCameraParameters &&
        m_previouslySentMessages.viewportCameraParameters != m_params.viewportCameraParameters) {
      sei.messages().emplace_back(MivBitstream::PayloadType::viewport_camera_parameters,
                                  MivBitstream::SeiPayload{*m_params.viewportCameraParameters});
      m_previouslySentMessages.viewportCameraParameters = m_params.viewportCameraParameters;
    }

    if (m_params.viewportPosition &&
        m_previouslySentMessages.viewportPosition != m_params.viewportPosition) {
      sei.messages().emplace_back(MivBitstream::PayloadType::viewport_position,
                                  MivBitstream::SeiPayload{*m_params.viewportPosition});
      m_previouslySentMessages.viewportPosition = m_params.viewportPosition;
    }

    if (!sei.messages().empty()) {
      encodeSeiRbspToAsb(asb, sei, nuhPrefixNsei);
    }
  }

  static void encodeSuffixSeiMessages(MivBitstream::AtlasSubBitstream &asb) {
    std::vector<MivBitstream::SeiMessage> seiMessages{};
    if (!seiMessages.empty()) {
      MivBitstream::SeiRBSP seiRbsp{std::move(seiMessages)};
      encodeSeiRbspToAsb(asb, seiRbsp, nuhSuffixNsei);
    }
  }

  void checkBitstreamRequirements() {
    const auto vme_decoder_side_depth_estimation_flag =
        m_params.vps.vpsMiv2ExtensionPresentFlag() &&
        m_params.vps.vps_miv_2_extension().vme_decoder_side_depth_estimation_flag();

    const auto casme_decoder_side_depth_estimation_flag =
        m_params.casps.casps_miv_2_extension_present_flag() &&
        m_params.casps.casps_miv_2_extension().casme_decoder_side_depth_estimation_flag();

    PRECONDITION(vme_decoder_side_depth_estimation_flag ==
                 casme_decoder_side_depth_estimation_flag);

    for (uint8_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
      const auto j = m_params.vps.vps_atlas_id(k);
      const auto &atlas = m_params.atlas[k];
      VERIFY_V3CBITSTREAM(m_params.vps.vps_frame_width(j) == atlas.asps.asps_frame_width());
      VERIFY_V3CBITSTREAM(m_params.vps.vps_frame_height(j) == atlas.asps.asps_frame_height());
      VERIFY_V3CBITSTREAM(m_params.vps.vps_map_count_minus1(j) ==
                          atlas.asps.asps_map_count_minus1());
    }
  }

  std::function<void(std::optional<V3cUnit>)> m_sink;
  bool m_rewriteParameterSets;

  State m_state{State::initial};
  EncoderParams m_params;
  PreviouslySentMessages m_previouslySentMessages{};
  uint8_t m_log2MaxFrmOrderCntLsbMinus4{};
  uint16_t m_frmOrderCntLsb{};
  Common::FlatMap<MivBitstream::V3cUnitHeader, AtlasSubBitstream> m_asbBuffers;
};
} // namespace

auto encodeMiv(Common::Sink<MivBitstream::V3cUnit> sink, bool rewriteParameterSets)
    -> Common::Sink<EncoderParams> {
  return [encoder = std::make_shared<MivEncoder>(std::move(sink), rewriteParameterSets)](
             std::optional<EncoderParams> frame) { return (*encoder)(std::move(frame)); };
}
} // namespace TMIV::Encoder
