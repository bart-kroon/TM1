/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#include <TMIV/PtlChecker/PtlChecker.h>

// A macro is used to capture the text of the condition. There is no reflection in C++17.
//
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define PTL_CHECK(document, numberedItem, condition)                                               \
  if (!(condition)) {                                                                              \
    m_logger(fmt::format("{} [{} {}]", #condition, document, numberedItem));                       \
  }

namespace TMIV::PtlChecker {
static constexpr auto miv1 = "ISO/IEC 23090-12:2021";
static constexpr auto v3c2dis = "ISO/IEC DIS 23090-5(2E):2021";

using CF = Common::ColorFormat;
using CG = MivBitstream::PtlProfileCodecGroupIdc;
using TS = MivBitstream::PtlProfileToolsetIdc;
using RC = MivBitstream::PtlProfileReconstructionIdc;
using LV = MivBitstream::PtlLevelIdc;
using VUT = MivBitstream::VuhUnitType;
using NUT = MivBitstream::NalUnitType;
using ATI = MivBitstream::AiAttributeTypeId;
using APM = MivBitstream::AtduPatchMode;

using Common::contains;

void PtlChecker::replaceLogger(Logger value) {
  PRECONDITION(value);

  m_logger = std::move(value);
}

void PtlChecker::defaultLogger(const std::string &failure) {
  fmt::print("WARNING: A profile-tier-level check has failed: {}\n"
             "         From this point onwards behaviour is undefined."
             " Do not report any subsequent errors.\n",
             failure);
}

auto PtlChecker::ptl_profile_codec_group_idc() const noexcept {
  return m_vps->profile_tier_level().ptl_profile_codec_group_idc();
}

auto PtlChecker::ptl_profile_toolset_idc() const noexcept {
  return m_vps->profile_tier_level().ptl_profile_toolset_idc();
}

auto PtlChecker::ptl_profile_reconstruction_idc() const noexcept {
  return m_vps->profile_tier_level().ptl_profile_reconstruction_idc();
}

auto PtlChecker::ptl_tier_flag() const noexcept {
  return m_vps->profile_tier_level().ptl_tier_flag();
}

auto PtlChecker::ptl_level_idc() const noexcept {
  return m_vps->profile_tier_level().ptl_level_idc();
}

auto PtlChecker::ptc_restricted_geometry_flag() const noexcept {
  const auto &ptl = m_vps->profile_tier_level();

  if (ptl.ptl_toolset_constraints_present_flag()) {
    const auto &ptc = ptl.ptl_profile_toolset_constraints_information();
    return ptc.ptc_restricted_geometry_flag();
  }

  return false;
}

auto PtlChecker::maxAtlasSize() const noexcept {
  const auto level = ptl_level_idc();

  if (level <= LV::Level_1_5) {
    return 2'228'224;
  }
  if (level <= LV::Level_2_5) {
    return 8'912'896;
  }
  if (level <= LV::Level_3_5) {
    return 35'651'584;
  }
  if (level <= LV::Level_4_5) {
    return 134'217'728;
  }
  return INT32_MAX;
}

auto PtlChecker::levelMapCount() const noexcept {
  const auto level = ptl_level_idc();

  if (level <= LV::Level_1_5) {
    return 2;
  }
  if (level <= LV::Level_2_5) {
    return 4;
  }
  if (level <= LV::Level_3_5) {
    return 8;
  }
  if (level <= LV::Level_4_5) {
    return 16;
  }
  return 64;
}

auto PtlChecker::maxNumAttributeCount() const noexcept {
  const auto level = ptl_level_idc();

  if (level <= LV::Level_1_0) {
    return 1;
  }
  if (level <= LV::Level_1_5) {
    return 3;
  }
  if (level <= LV::Level_2_0) {
    return 4;
  }
  if (level <= LV::Level_2_5) {
    return 8;
  }
  if (level <= LV::Level_3_0) {
    return 16;
  }
  if (level <= LV::Level_3_5) {
    return 24;
  }
  if (level <= LV::Level_4_0) {
    return 32;
  }
  if (level <= LV::Level_4_5) {
    return 48;
  }
  return 63;
}

void PtlChecker::checkVuh(const MivBitstream::V3cUnitHeader &vuh) {
  PRECONDITION(vuh.vuh_unit_type() == VUT::V3C_VPS || m_vps.has_value());

  if (vuh.vuh_unit_type() == VUT::V3C_VPS) {
    return;
  }

  switch (ptl_profile_toolset_idc()) {
  case TS::VPCC_Basic:
  case TS::VPCC_Extended:
    PTL_CHECK(v3c2dis, "Table H-3",
              contains(std::array{VUT::V3C_AD, VUT::V3C_OVD, VUT::V3C_GVD, VUT::V3C_AVD},
                       vuh.vuh_unit_type()));
    break;
  case TS::MIV_Main:
    PTL_CHECK(miv1, "Table A-1",
              contains(std::array{VUT::V3C_AD, VUT::V3C_GVD, VUT::V3C_AVD, VUT::V3C_CAD},
                       vuh.vuh_unit_type()));
    break;
  case TS::MIV_Extended:
    if (ptc_restricted_geometry_flag()) {
      PTL_CHECK(miv1, "Table A-1",
                contains(std::array{VUT::V3C_AD, VUT::V3C_AVD, VUT::V3C_PVD, VUT::V3C_CAD},
                         vuh.vuh_unit_type()));
    } else {
      PTL_CHECK(miv1, "Table A-1",
                contains(std::array{VUT::V3C_AD, VUT::V3C_OVD, VUT::V3C_GVD, VUT::V3C_AVD,
                                    VUT::V3C_PVD, VUT::V3C_CAD},
                         vuh.vuh_unit_type()));
    }
    break;
  case TS::MIV_Geometry_Absent:
    PTL_CHECK(miv1, "Table A-1",
              contains(std::array{VUT::V3C_AD, VUT::V3C_AVD, VUT::V3C_PVD, VUT::V3C_CAD},
                       vuh.vuh_unit_type()));
    break;
  }
}

void PtlChecker::checkNuh(const MivBitstream::NalUnitHeader &nuh) {
  PTL_CHECK(miv1, "A.1", nuh.nal_temporal_id_plus1() == 1);
}

void PtlChecker::checkAndActivateVps(const MivBitstream::V3cParameterSet &vps) {
  m_vps = vps;

  checkVpsCommon(vps);

  for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
    checkVpsAtlas(vps, vps.vps_atlas_id(k));
  }

  if (vps.vps_miv_extension_present_flag()) {
    checkVpsMivExtension(vps.vps_miv_extension());
  }
}

void PtlChecker::checkVpsCommon(const MivBitstream::V3cParameterSet &vps) const {
  PTL_CHECK(v3c2dis, "Table A-1",
            contains(MivBitstream::knownCodecGroupIdcs, ptl_profile_codec_group_idc()));
  PTL_CHECK(v3c2dis, "Table A-3",
            contains(MivBitstream::knownToolsetIdcs, ptl_profile_toolset_idc()));
  PTL_CHECK(v3c2dis, "Table H-4",
            contains(MivBitstream::knownReconstructionIdcs, ptl_profile_reconstruction_idc()));
  PTL_CHECK(v3c2dis, "A.6.2, Table A-5", contains(MivBitstream::knownLevelIdcs, ptl_level_idc()));

  PTL_CHECK(v3c2dis, "?", !ptl_tier_flag());

  const auto vps_miv_extension_present_flag = vps.vps_miv_extension_present_flag();
  const auto vps_packing_information_present_flag = vps.vps_packing_information_present_flag();
  const auto vps_atlas_count_minus1 = vps.vps_atlas_count_minus1();

  switch (ptl_profile_toolset_idc()) {
  case TS::VPCC_Basic:
  case TS::VPCC_Extended:
    PTL_CHECK(v3c2dis, "Table H-3", !vps_miv_extension_present_flag);
    PTL_CHECK(v3c2dis, "Table H-3", !vps_packing_information_present_flag);
    PTL_CHECK(v3c2dis, "Table H-3", vps_atlas_count_minus1 == 0);
    break;
  case TS::MIV_Main:
    PTL_CHECK(miv1, "Table A-1", ptl_profile_reconstruction_idc() == RC::Rec_Unconstrained);
    PTL_CHECK(miv1, "Table A-1", vps_miv_extension_present_flag);
    PTL_CHECK(miv1, "Table A-1", !vps_packing_information_present_flag);
    break;
  case TS::MIV_Extended:
  case TS::MIV_Geometry_Absent:
    PTL_CHECK(miv1, "Table A-1", ptl_profile_reconstruction_idc() == RC::Rec_Unconstrained);
    PTL_CHECK(miv1, "Table A-1", vps_miv_extension_present_flag);
    break;
  }
}

void PtlChecker::checkVpsAtlas(const MivBitstream::V3cParameterSet &vps,
                               MivBitstream::AtlasId atlasId) const {
  const auto vps_map_count_minus1 = vps.vps_map_count_minus1(atlasId);
  const auto vps_occupancy_video_present_flag = vps.vps_occupancy_video_present_flag(atlasId);
  const auto vps_geometry_video_present_flag = vps.vps_geometry_video_present_flag(atlasId);

  PTL_CHECK(v3c2dis, "Table A-5", vps_map_count_minus1 < levelMapCount());

  switch (ptl_profile_toolset_idc()) {
  case TS::VPCC_Basic:
    PTL_CHECK(v3c2dis, "Table H-3", vps_map_count_minus1 <= 1);
    PTL_CHECK(v3c2dis, "Table H-3", vps_occupancy_video_present_flag);
    PTL_CHECK(v3c2dis, "Table H-3", vps_geometry_video_present_flag);
    break;
  case TS::VPCC_Extended:
    PTL_CHECK(v3c2dis, "Table H-3", vps_occupancy_video_present_flag);
    PTL_CHECK(v3c2dis, "Table H-3", vps_geometry_video_present_flag);
    break;
  case TS::MIV_Main:
    PTL_CHECK(miv1, "Table A-1", vps_map_count_minus1 == 0);
    PTL_CHECK(miv1, "Table A-1", !vps_occupancy_video_present_flag);
    PTL_CHECK(miv1, "Table A-1", vps_geometry_video_present_flag);
    break;
  case TS::MIV_Extended:
    PTL_CHECK(miv1, "Table A-1", vps_map_count_minus1 == 0);

    if (ptc_restricted_geometry_flag()) {
      PTL_CHECK(miv1, "Table A-1", !vps_occupancy_video_present_flag);
      PTL_CHECK(miv1, "Table A-1", !vps_geometry_video_present_flag);
    }
    break;
  case TS::MIV_Geometry_Absent:
    PTL_CHECK(miv1, "Table A-1", vps_map_count_minus1 == 0);
    PTL_CHECK(miv1, "Table A-1", !vps_occupancy_video_present_flag);
    PTL_CHECK(miv1, "Table A-1", !vps_geometry_video_present_flag);
    break;
  }

  if (vps.vps_geometry_video_present_flag(atlasId)) {
    checkGeometryInformation(vps.geometry_information(atlasId));
  }

  if (vps.vps_attribute_video_present_flag(atlasId)) {
    checkAttributesInformation(vps.attribute_information(atlasId));
  }
}

void PtlChecker::checkGeometryInformation(const MivBitstream::GeometryInformation &gi) const {
  const auto gi_geometry_MSB_align_flag = gi.gi_geometry_MSB_align_flag();

  switch (ptl_profile_toolset_idc()) {
  case TS::VPCC_Basic:
  case TS::VPCC_Extended:
    break;
  case TS::MIV_Main:
  case TS::MIV_Extended:
  case TS::MIV_Geometry_Absent:
    PTL_CHECK(miv1, "Table A-1", !gi_geometry_MSB_align_flag);
    break;
  }
}

void PtlChecker::checkAttributesInformation(const MivBitstream::AttributeInformation &ai) const {
  const auto ai_attribute_count = ai.ai_attribute_count();

  PTL_CHECK(v3c2dis, "Table A-5", ai_attribute_count <= maxNumAttributeCount());

  switch (ptl_profile_toolset_idc()) {
  case TS::VPCC_Basic:
  case TS::VPCC_Extended:
    break;
  case TS::MIV_Main:
    PTL_CHECK(miv1, "Table A-1", ai_attribute_count <= 1);
    break;
  case TS::MIV_Extended:
    if (ptc_restricted_geometry_flag()) {
      PTL_CHECK(miv1, "Table A-1", ai_attribute_count == 2);

      if (ai_attribute_count == 2) {
        PTL_CHECK(miv1, "Table A-1", ai.ai_attribute_type_id(0) != ai.ai_attribute_type_id(1));
      }
    } else {
      PTL_CHECK(miv1, "Table A-1", ai_attribute_count <= 2);
    }
    break;
  case TS::MIV_Geometry_Absent:
    PTL_CHECK(miv1, "Table A-1", ai_attribute_count <= 1);
    break;
  }

  for (uint8_t i = 0; i < ai_attribute_count; ++i) {
    checkAttributeInformation(ai, i);
  }
}

void PtlChecker::checkAttributeInformation(const MivBitstream::AttributeInformation &ai,
                                           uint8_t attrIdx) const {
  const auto ai_attribute_type_id = ai.ai_attribute_type_id(attrIdx);
  const auto ai_attribute_dimension_minus1 = ai.ai_attribute_dimension_minus1(attrIdx);
  const auto ai_attribute_MSB_align_flag = ai.ai_attribute_MSB_align_flag(attrIdx);

  PTL_CHECK(v3c2dis, "8.4.4.5", ai_attribute_type_id != ATI::ATTR_UNSPECIFIED);
  PTL_CHECK(v3c2dis, "8.4.4.5",
            ai_attribute_type_id != ATI::ATTR_NORMAL || ai_attribute_dimension_minus1 == 2);

  switch (ptl_profile_toolset_idc()) {
  case TS::VPCC_Basic:
    PTL_CHECK(v3c2dis, "Table H-3", ai_attribute_dimension_minus1 == 2);
    break;
  case TS::VPCC_Extended:
    break;
  case TS::MIV_Main:
  case TS::MIV_Geometry_Absent:
    PTL_CHECK(miv1, "Table A-1", ai_attribute_type_id == ATI::ATTR_TEXTURE);
    PTL_CHECK(miv1, "Table A-1", !ai_attribute_MSB_align_flag);

    if (ai_attribute_type_id == ATI::ATTR_TEXTURE) {
      PTL_CHECK(miv1, "Table A-1", ai_attribute_dimension_minus1 == 2);
    }
    break;
  case TS::MIV_Extended:
    PTL_CHECK(miv1, "Table A-1",
              ai_attribute_type_id == ATI::ATTR_TEXTURE ||
                  ai_attribute_type_id == ATI::ATTR_TRANSPARENCY);
    PTL_CHECK(miv1, "Table A-1", !ai_attribute_MSB_align_flag);

    if (ai_attribute_type_id == ATI::ATTR_TEXTURE) {
      PTL_CHECK(miv1, "Table A-1", ai_attribute_dimension_minus1 == 2);
    } else if (ai_attribute_type_id == ATI::ATTR_TRANSPARENCY) {
      PTL_CHECK(miv1, "Table A-1", ai_attribute_dimension_minus1 == 0);
    }
    break;
  }
}

void PtlChecker::checkVpsMivExtension(const MivBitstream::VpsMivExtension &vme) const {
  const auto vme_embedded_occupancy_enabled_flag = vme.vme_embedded_occupancy_enabled_flag();

  switch (ptl_profile_toolset_idc()) {
  case TS::VPCC_Basic:
  case TS::VPCC_Extended:
    break;
  case TS::MIV_Main:
    PTL_CHECK(miv1, "Table A-1", vme_embedded_occupancy_enabled_flag);
    break;
  case TS::MIV_Extended:
    if (ptc_restricted_geometry_flag()) {
      PTL_CHECK(miv1, "Table A-1", !vme_embedded_occupancy_enabled_flag);
    }
    break;
  case TS::MIV_Geometry_Absent:
    PTL_CHECK(miv1, "Table A-1", !vme_embedded_occupancy_enabled_flag);
    break;
  }
}

void PtlChecker::checkAsps(MivBitstream::AtlasId atlasId,
                           const MivBitstream::AtlasSequenceParameterSetRBSP &asps) {
  PRECONDITION(m_vps.has_value());

  switch (ptl_profile_toolset_idc()) {
  case TS::VPCC_Basic:
    PTL_CHECK(v3c2dis, "Table H-3", !asps.asps_eom_patch_enabled_flag());
    PTL_CHECK(v3c2dis, "Table H-3", !asps.asps_plr_enabled_flag());
    PTL_CHECK(v3c2dis, "Table H-3", !asps.asps_use_eight_orientations_flag());
    PTL_CHECK(v3c2dis, "Table H-3", !asps.asps_extended_projection_enabled_flag());
    PTL_CHECK(v3c2dis, "Table H-3", !asps.asps_miv_extension_present_flag());
    break;
  case TS::VPCC_Extended:
    PTL_CHECK(v3c2dis, "Table H-3", !asps.asps_miv_extension_present_flag());
    break;
  case TS::MIV_Main:
  case TS::MIV_Extended:
  case TS::MIV_Geometry_Absent:
    PTL_CHECK(miv1, "Table A-1", !asps.asps_long_term_ref_atlas_frames_flag());
    PTL_CHECK(miv1, "Table A-1", !asps.asps_pixel_deinterleaving_enabled_flag());
    PTL_CHECK(miv1, "Table A-1", !asps.asps_patch_precedence_order_flag());
    PTL_CHECK(miv1, "Table A-1", !asps.asps_raw_patch_enabled_flag());
    PTL_CHECK(miv1, "Table A-1", !asps.asps_eom_patch_enabled_flag());
    PTL_CHECK(miv1, "Table A-1", !asps.asps_plr_enabled_flag());
  }

  if (ptc_restricted_geometry_flag()) {
    // Because of asme_patch_constant_depth_flag, the ASME is required
    PTL_CHECK(miv1, "Table A-1 (implicit)", asps.asps_miv_extension_present_flag());
  }

  if (asps.asps_miv_extension_present_flag()) {
    checkAsme(atlasId, asps.asps_miv_extension());
  }

  const auto aspsFrameSize = asps.asps_frame_width() * asps.asps_frame_height();
  PTL_CHECK(v3c2dis, "Table A-6", aspsFrameSize <= maxAtlasSize())
}

void PtlChecker::checkAsme(MivBitstream::AtlasId atlasId,
                           const MivBitstream::AspsMivExtension &asme) const {
  const auto vps_geometry_video_present_flag = m_vps->vps_geometry_video_present_flag(atlasId);
  const auto asme_patch_constant_depth_flag = asme.asme_patch_constant_depth_flag();
  const auto pin_geometry_present_flag =
      m_vps->vps_packed_video_present_flag(atlasId)
          ? m_vps->packing_information(atlasId).pin_geometry_present_flag()
          : false;

  switch (ptl_profile_toolset_idc()) {
  case TS::MIV_Main:
    PTL_CHECK(miv1, "Table A-1", !asme_patch_constant_depth_flag);
    break;
  case TS::MIV_Extended:
    // Circumvent spec issue http://mpegx.int-evry.fr/software/MPEG/MIV/Specs/23090-12/-/issues/436
    PTL_CHECK(miv1, "Table A-1",
              vps_geometry_video_present_flag || pin_geometry_present_flag ||
                  asme_patch_constant_depth_flag);
    break;
  case TS::VPCC_Basic:
  case TS::VPCC_Extended:
  case TS::MIV_Geometry_Absent:
    break;
  }
}

void PtlChecker::checkAfps(const MivBitstream::AtlasFrameParameterSetRBSP &afps) {
  PTL_CHECK(miv1, "Table A-1", !afps.afps_lod_mode_enabled_flag());
  PTL_CHECK(miv1, "Table A-1", !afps.afps_raw_3d_offset_bit_count_explicit_mode_flag());
}

void PtlChecker::checkAtl(const MivBitstream::NalUnitHeader &nuh,
                          const MivBitstream::AtlasTileLayerRBSP &atl) {
  const auto &ath = atl.atlas_tile_header();
  const auto ath_type = ath.ath_type();

  switch (ptl_profile_toolset_idc()) {
  case TS::VPCC_Basic:
  case TS::VPCC_Extended:
    break;
  case TS::MIV_Main:
  case TS::MIV_Extended:
  case TS::MIV_Geometry_Absent:
    PTL_CHECK(miv1, "Table A-1", ath_type == MivBitstream::AthType::I_TILE);

    atl.atlas_tile_data_unit().visit([this](size_t /* p */, APM atdu_patch_mode,
                                            const MivBitstream::PatchInformationData & /* pid */) {
      PTL_CHECK(miv1, "Table A-1", atdu_patch_mode == APM::I_INTRA);
    });
    break;
  }

  const auto idrCodedAtlas = contains(
      std::array{NUT::NAL_IDR_W_RADL, NUT::NAL_IDR_N_LP, NUT::NAL_GIDR_W_RADL, NUT::NAL_GIDR_N_LP},
      nuh.nal_unit_type());
  PTL_CHECK(v3c2dis, "A.6.1", !idrCodedAtlas || ath.ath_atlas_frm_order_cnt_lsb() == 0);
}

void PtlChecker::checkCaf(const MivBitstream::NalUnitHeader &nuh,
                          const MivBitstream::CommonAtlasFrameRBSP &caf) {
  const auto irapCodedCommonAtlas = nuh.nal_unit_type() == NUT::NAL_CAF_IDR;
  PTL_CHECK(v3c2dis, "A.6.1",
            !irapCodedCommonAtlas || caf.caf_common_atlas_frm_order_cnt_lsb() == 0);
}

void PtlChecker::checkVideoFrame(VUT vut, const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                                 const Common::Frame<> &frame) {
  PRECONDITION(!frame.empty());
  PRECONDITION(m_vps.has_value());

  const auto ptl_profile_codec_group_idc =
      m_vps->profile_tier_level().ptl_profile_codec_group_idc();

  if (ptl_profile_codec_group_idc == CG::MP4RA ||
      !contains(MivBitstream::knownCodecGroupIdcs, ptl_profile_codec_group_idc)) {
    return;
  }

  const auto colorFormat = frame.getColorFormat();
  PTL_CHECK(v3c2dis, "Table A-2",
            colorFormat == CF::YUV420 ||
                (colorFormat == CF::YUV400 && (ptl_profile_codec_group_idc == CG::HEVC444 ||
                                               ptl_profile_codec_group_idc == CG::VVC_Main10)) ||
                (colorFormat == CF::YUV444 && vut == VUT::V3C_AVD &&
                 ptl_profile_codec_group_idc == CG::HEVC444));

  const auto bitDepth = frame.getBitDepth();
  PTL_CHECK(v3c2dis, "Table A-2",
            bitDepth == 8 || (bitDepth == 10 && (ptl_profile_codec_group_idc == CG::HEVC_Main10 ||
                                                 ptl_profile_codec_group_idc == CG::HEVC444 ||
                                                 ptl_profile_codec_group_idc == CG::VVC_Main10)));

  switch (vut) {
  case VUT::V3C_OVD:
    return checkOccupancyVideoFrame(asps, frame);
  case VUT::V3C_GVD:
    return checkGeometryVideoFrame(asps, frame);
  case VUT::V3C_AVD:
    return checkAttributeVideoFrame(asps, frame);
  case VUT::V3C_PVD:
    // Specification issue http://mpegx.int-evry.fr/software/MPEG/PCC/Specs/23090-5/-/issues/496
    break;
  default:
    UNREACHABLE;
  }
}

void PtlChecker::checkOccupancyVideoFrame(const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                                          const Common::Frame<> &frame) const {
  int32_t occScaleX = 1;
  int32_t occScaleY = 1;

  if (m_vps->vps_miv_extension_present_flag() &&
      m_vps->vps_miv_extension().vme_occupancy_scale_enabled_flag()) {
    const auto &asme = asps.asps_miv_extension();

    occScaleX = asme.asme_occupancy_scale_factor_x_minus1() + 1;
    occScaleY = asme.asme_occupancy_scale_factor_y_minus1() + 1;
  }

  const auto asmeOccupancyFrameWidth = asps.asps_frame_width() / occScaleX;
  const auto asmeOccupancyFrameHeight = asps.asps_frame_height() / occScaleY;

  PTL_CHECK(miv1, "A.4.1", frame.getWidth() == asmeOccupancyFrameWidth);
  PTL_CHECK(miv1, "A.4.1", frame.getHeight() == asmeOccupancyFrameHeight);
}

void PtlChecker::checkGeometryVideoFrame(const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                                         const Common::Frame<> &frame) const {
  int32_t geoScaleX = 1;
  int32_t geoScaleY = 1;

  if (m_vps->vps_miv_extension_present_flag() &&
      m_vps->vps_miv_extension().vme_geometry_scale_enabled_flag()) {
    const auto &asme = asps.asps_miv_extension();

    geoScaleX = asme.asme_geometry_scale_factor_x_minus1() + 1;
    geoScaleY = asme.asme_geometry_scale_factor_y_minus1() + 1;
  }

  const auto asmeGeometryFrameWidth = asps.asps_frame_width() / geoScaleX;
  const auto asmeGeometryFrameHeight = asps.asps_frame_height() / geoScaleY;

  PTL_CHECK(miv1, "A.4.1", frame.getWidth() == asmeGeometryFrameWidth);
  PTL_CHECK(miv1, "A.4.1", frame.getHeight() == asmeGeometryFrameHeight);
}

void PtlChecker::checkAttributeVideoFrame(const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                                          const Common::Frame<> &frame) const {
  const auto aspsFrameWidth = asps.asps_frame_width();
  const auto aspsFrameHeight = asps.asps_frame_height();

  PTL_CHECK(miv1, "A.4.1", frame.getWidth() == aspsFrameWidth);
  PTL_CHECK(miv1, "A.4.1", frame.getHeight() == aspsFrameHeight);
}
} // namespace TMIV::PtlChecker
