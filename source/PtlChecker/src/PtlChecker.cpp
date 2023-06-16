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

#include <TMIV/PtlChecker/PtlChecker.h>

#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/MivBitstream/Formatters.h>

namespace TMIV::PtlChecker {
using Common::contains;
using Common::downCast;
using Common::logVerbose;

struct PtlChecker::Impl {
  struct V3cFrameVariables {
    int64_t numProjPatches;
    int64_t numLumaSamples;
  };

  Logger m_logger{&defaultLogger};
  std::optional<MivBitstream::V3cParameterSet> m_vps;
  bool m_haveV3cFrame{};
  double m_frameRate{30.};
  std::vector<V3cFrameVariables> m_window;

  void ptlCheck(bool condition, const char *what, const char *document,
                const char *numberedItem) const {
    if (!condition) {
      m_logger(fmt::format("{} [{} {}]", what, document, numberedItem));
    }
  }

  // A macro is used to capture the text of the condition. There is no reflection in C++17.
#define PTL_CHECK(document, numberedItem, condition)                                               \
  ptlCheck(condition, #condition, document, numberedItem)

  template <typename Value, typename Limit>
  auto levelCheck(const char *document, const char *numberedItem, const char *identifier,
                  Value value, Limit limit) const {
    using Integer = std::common_type_t<Value, Limit>;

    if (Integer{value} <= Integer{limit}) {
      logVerbose("Level check: {} <= {} = {}", value, identifier, limit);
    } else {
      m_logger(
          fmt::format("{} <= {} = {} [{} {}]", value, identifier, limit, document, numberedItem));
    }
    return value;
  }

  static constexpr auto v3cSpec = "ISO/IEC 23090-5(2E)/Amd.1";
  static constexpr auto mivSpec = "ISO/IEC 23090-12/Amd.1";

  using CF = Common::ColorFormat;
  using CG = MivBitstream::PtlProfileCodecGroupIdc;
  using TS = MivBitstream::PtlProfileToolsetIdc;
  using RC = MivBitstream::PtlProfileReconstructionIdc;
  using LV = MivBitstream::PtlLevelIdc;
  using VUT = MivBitstream::VuhUnitType;
  using NUT = MivBitstream::NalUnitType;
  using ATI = MivBitstream::AiAttributeTypeId;
  using APM = MivBitstream::AtduPatchMode;

  void replaceLogger(Logger value) {
    PRECONDITION(value);

    m_logger = std::move(value);
  }

  static void defaultLogger(const std::string &failure) {
    Common::logWarning(
        "A profile-tier-level check has failed: {}. From this point onwards behaviour "
        "is undefined. Do not report any subsequent errors.",
        failure);
  }

  [[nodiscard]] auto ptl_profile_codec_group_idc() const noexcept {
    return m_vps->profile_tier_level().ptl_profile_codec_group_idc();
  }

  [[nodiscard]] auto ptl_profile_toolset_idc() const noexcept {
    return m_vps->profile_tier_level().ptl_profile_toolset_idc();
  }

  [[nodiscard]] auto ptl_profile_reconstruction_idc() const noexcept {
    return m_vps->profile_tier_level().ptl_profile_reconstruction_idc();
  }

  [[nodiscard]] auto ptl_tier_flag() const noexcept {
    return m_vps->profile_tier_level().ptl_tier_flag();
  }

  [[nodiscard]] auto ptl_level_idc() const noexcept {
    return m_vps->profile_tier_level().ptl_level_idc();
  }

  [[nodiscard]] auto ptc_restricted_geometry_flag() const noexcept {
    const auto &ptl = m_vps->profile_tier_level();

    if (ptl.ptl_toolset_constraints_present_flag()) {
      const auto &ptc = ptl.ptl_profile_toolset_constraints_information();
      return ptc.ptc_restricted_geometry_flag();
    }

    return false;
  }

  [[nodiscard]] auto ptc_one_v3c_frame_only_flag() const noexcept {
    const auto &ptl = m_vps->profile_tier_level();

    if (ptl.ptl_toolset_constraints_present_flag()) {
      const auto &ptc = ptl.ptl_profile_toolset_constraints_information();
      return ptc.ptc_one_v3c_frame_only_flag();
    }
    return false;
  }

  template <typename... Integer> [[nodiscard]] auto levelLimit(Integer... limit) const {
    return levelLimit(std::array<std::common_type_t<Integer...>, sizeof...(Integer)>{limit...});
  }

  template <typename Integer>
  [[nodiscard]] auto levelLimit(const std::array<Integer, 4> &limits) const {
    const auto level = ptl_level_idc();

    if (level <= LV::Level_1_5) {
      return limits[0];
    }
    if (level <= LV::Level_2_5) {
      return limits[1];
    }
    if (level <= LV::Level_3_5) {
      return limits[2];
    }
    if (level <= LV::Level_4_5) {
      return limits[3];
    }
    return std::numeric_limits<Integer>::max();
  }

  template <typename Integer>
  [[nodiscard]] auto levelLimit(const std::array<Integer, 8> &limits) const {
    const auto level = ptl_level_idc();

    if (level <= LV::Level_1_0) {
      return limits[0];
    }
    if (level <= LV::Level_1_5) {
      return limits[1];
    }
    if (level <= LV::Level_2_0) {
      return limits[2];
    }
    if (level <= LV::Level_2_5) {
      return limits[3];
    }
    if (level <= LV::Level_3_0) {
      return limits[4];
    }
    if (level <= LV::Level_3_5) {
      return limits[5];
    }
    if (level <= LV::Level_4_0) {
      return limits[6];
    }
    if (level <= LV::Level_4_5) {
      return limits[7];
    }
    return std::numeric_limits<Integer>::max();
  }

  void checkVuh(const MivBitstream::V3cUnitHeader &vuh) {
    PRECONDITION(vuh.vuh_unit_type() == VUT::V3C_VPS || m_vps.has_value());

    if (vuh.vuh_unit_type() == VUT::V3C_VPS) {
      return;
    }

    switch (ptl_profile_toolset_idc()) {
    case TS::VPCC_Basic:
    case TS::VPCC_Extended:
      PTL_CHECK(v3cSpec, "Table H-3",
                contains(std::array{VUT::V3C_AD, VUT::V3C_OVD, VUT::V3C_GVD, VUT::V3C_AVD},
                         vuh.vuh_unit_type()));
      break;
    case TS::MIV_Main:
      PTL_CHECK(mivSpec, "Table A-1",
                contains(std::array{VUT::V3C_AD, VUT::V3C_GVD, VUT::V3C_AVD, VUT::V3C_CAD},
                         vuh.vuh_unit_type()));
      break;
    case TS::MIV_Extended:
      if (ptc_restricted_geometry_flag()) {
        PTL_CHECK(mivSpec, "Table A-1",
                  contains(std::array{VUT::V3C_AD, VUT::V3C_AVD, VUT::V3C_PVD, VUT::V3C_CAD},
                           vuh.vuh_unit_type()));
      } else {
        PTL_CHECK(mivSpec, "Table A-1",
                  contains(std::array{VUT::V3C_AD, VUT::V3C_OVD, VUT::V3C_GVD, VUT::V3C_AVD,
                                      VUT::V3C_PVD, VUT::V3C_CAD},
                           vuh.vuh_unit_type()));
      }
      break;
    case TS::MIV_Geometry_Absent:
      PTL_CHECK(mivSpec, "Table A-1",
                contains(std::array{VUT::V3C_AD, VUT::V3C_AVD, VUT::V3C_PVD, VUT::V3C_CAD},
                         vuh.vuh_unit_type()));
      break;
    }
  }

  void checkNuh(const MivBitstream::NalUnitHeader &nuh) const {
    PTL_CHECK(mivSpec, "A.1", nuh.nal_temporal_id_plus1() == 1);
  }

  void checkAndActivateVps(const MivBitstream::V3cParameterSet &vps) {
    m_vps = vps;

    checkVpsCommon(vps);

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      checkVpsAtlas(vps, vps.vps_atlas_id(k));
    }

    if (vps.vpsMivExtensionPresentFlag()) {
      checkVpsMivExtension(vps.vps_miv_extension());
    }
  }

  void checkVpsCommon(const MivBitstream::V3cParameterSet &vps) const {
    PTL_CHECK(v3cSpec, "Table A-1",
              contains(MivBitstream::knownCodecGroupIdcs, ptl_profile_codec_group_idc()));
    PTL_CHECK(v3cSpec, "Table A-3",
              contains(MivBitstream::knownToolsetIdcs, ptl_profile_toolset_idc()));
    PTL_CHECK(v3cSpec, "Table H-4",
              contains(MivBitstream::knownReconstructionIdcs, ptl_profile_reconstruction_idc()));
    PTL_CHECK(mivSpec, "A.4.3", ptl_level_idc() != LV::Level_8_5 || ptc_one_v3c_frame_only_flag());
    PTL_CHECK(v3cSpec, "A.6.2, Table A-5", contains(MivBitstream::knownLevelIdcs, ptl_level_idc()));

    PTL_CHECK(v3cSpec, "?", !ptl_tier_flag());

    const auto vpsMivExtensionPresentFlag = vps.vpsMivExtensionPresentFlag();
    const auto vpsPackingInformationPresentFlag = vps.vpsPackingInformationPresentFlag();
    const auto vps_atlas_count_minus1 = vps.vps_atlas_count_minus1();

    switch (ptl_profile_toolset_idc()) {
    case TS::VPCC_Basic:
    case TS::VPCC_Extended:
      PTL_CHECK(v3cSpec, "Table H-3", !vpsMivExtensionPresentFlag);
      PTL_CHECK(v3cSpec, "Table H-3", !vpsPackingInformationPresentFlag);
      PTL_CHECK(v3cSpec, "Table H-3", vps_atlas_count_minus1 == 0);
      break;
    case TS::MIV_Main:
      PTL_CHECK(mivSpec, "Table A-1", ptl_profile_reconstruction_idc() == RC::Rec_Unconstrained);
      PTL_CHECK(mivSpec, "Table A-1", vpsMivExtensionPresentFlag);
      PTL_CHECK(mivSpec, "Table A-1", !vpsPackingInformationPresentFlag);
      break;
    case TS::MIV_Extended:
    case TS::MIV_Geometry_Absent:
      PTL_CHECK(mivSpec, "Table A-1", ptl_profile_reconstruction_idc() == RC::Rec_Unconstrained);
      PTL_CHECK(mivSpec, "Table A-1", vpsMivExtensionPresentFlag);
      break;
    }
  }

  void checkVpsAtlas(const MivBitstream::V3cParameterSet &vps,
                     MivBitstream::AtlasId atlasId) const {
    const auto vps_map_count_minus1 = vps.vps_map_count_minus1(atlasId);
    const auto vps_occupancy_video_present_flag = vps.vps_occupancy_video_present_flag(atlasId);
    const auto vps_geometry_video_present_flag = vps.vps_geometry_video_present_flag(atlasId);

    levelCheck(v3cSpec, "Table A-5", "LevelMapCount", vps_map_count_minus1 + 1,
               levelLimit(2, 2, 4, 4, 8, 8, 16, 16));

    switch (ptl_profile_toolset_idc()) {
    case TS::VPCC_Basic:
      PTL_CHECK(v3cSpec, "Table H-3", vps_map_count_minus1 <= 1);
      PTL_CHECK(v3cSpec, "Table H-3", vps_occupancy_video_present_flag);
      PTL_CHECK(v3cSpec, "Table H-3", vps_geometry_video_present_flag);
      break;
    case TS::VPCC_Extended:
      PTL_CHECK(v3cSpec, "Table H-3", vps_occupancy_video_present_flag);
      PTL_CHECK(v3cSpec, "Table H-3", vps_geometry_video_present_flag);
      break;
    case TS::MIV_Main:
      PTL_CHECK(mivSpec, "Table A-1", vps_map_count_minus1 == 0);
      PTL_CHECK(mivSpec, "Table A-1", !vps_occupancy_video_present_flag);
      PTL_CHECK(mivSpec, "Table A-1", vps_geometry_video_present_flag);
      break;
    case TS::MIV_Extended:
      PTL_CHECK(mivSpec, "Table A-1", vps_map_count_minus1 == 0);

      if (ptc_restricted_geometry_flag()) {
        PTL_CHECK(mivSpec, "Table A-1", !vps_occupancy_video_present_flag);
        PTL_CHECK(mivSpec, "Table A-1", !vps_geometry_video_present_flag);
      }
      break;
    case TS::MIV_Geometry_Absent:
      PTL_CHECK(mivSpec, "Table A-1", vps_map_count_minus1 == 0);
      PTL_CHECK(mivSpec, "Table A-1", !vps_occupancy_video_present_flag);
      PTL_CHECK(mivSpec, "Table A-1", !vps_geometry_video_present_flag);
      break;
    }

    if (vps.vps_geometry_video_present_flag(atlasId)) {
      checkGeometryInformation(vps.geometry_information(atlasId));
    }

    if (vps.vps_attribute_video_present_flag(atlasId)) {
      checkAttributesInformation(vps.attribute_information(atlasId));
    }
  }

  void checkGeometryInformation(const MivBitstream::GeometryInformation &gi) const {
    const auto gi_geometry_msb_align_flag = gi.gi_geometry_msb_align_flag();

    switch (ptl_profile_toolset_idc()) {
    case TS::VPCC_Basic:
    case TS::VPCC_Extended:
      break;
    case TS::MIV_Main:
    case TS::MIV_Extended:
    case TS::MIV_Geometry_Absent:
      PTL_CHECK(mivSpec, "Table A-1", !gi_geometry_msb_align_flag);
      break;
    }
  }

  void checkAttributesInformation(const MivBitstream::AttributeInformation &ai) const {
    const auto ai_attribute_count = ai.ai_attribute_count();

    levelCheck(v3cSpec, "Table A-5", "MaxNumAttributeCount", ai_attribute_count,
               levelLimit(1, 3, 4, 8, 16, 24, 32, 48));

    switch (ptl_profile_toolset_idc()) {
    case TS::VPCC_Basic:
    case TS::VPCC_Extended:
      break;
    case TS::MIV_Main:
      PTL_CHECK(mivSpec, "Table A-1", ai_attribute_count <= 1);
      break;
    case TS::MIV_Extended:
      if (ptc_restricted_geometry_flag()) {
        PTL_CHECK(mivSpec, "Table A-1", ai_attribute_count == 2);

        if (ai_attribute_count == 2) {
          PTL_CHECK(mivSpec, "Table A-1", ai.ai_attribute_type_id(0) != ai.ai_attribute_type_id(1));
        }
      } else {
        PTL_CHECK(mivSpec, "Table A-1", ai_attribute_count <= 2);
      }
      break;
    case TS::MIV_Geometry_Absent:
      PTL_CHECK(mivSpec, "Table A-1", ai_attribute_count <= 1);
      break;
    }

    for (uint8_t i = 0; i < ai_attribute_count; ++i) {
      checkAttributeInformation(ai, i);
    }
  }

  void checkAttributeInformation(const MivBitstream::AttributeInformation &ai,
                                 uint8_t attrIdx) const {
    const auto ai_attribute_type_id = ai.ai_attribute_type_id(attrIdx);
    const auto ai_attribute_dimension_minus1 = ai.ai_attribute_dimension_minus1(attrIdx);
    const auto ai_attribute_msb_align_flag = ai.ai_attribute_msb_align_flag(attrIdx);

    PTL_CHECK(v3cSpec, "8.4.4.5", ai_attribute_type_id != ATI::ATTR_UNSPECIFIED);
    PTL_CHECK(v3cSpec, "8.4.4.5",
              ai_attribute_type_id != ATI::ATTR_NORMAL || ai_attribute_dimension_minus1 == 2);

    switch (ptl_profile_toolset_idc()) {
    case TS::VPCC_Basic:
      PTL_CHECK(v3cSpec, "Table H-3", ai_attribute_dimension_minus1 == 2);
      break;
    case TS::VPCC_Extended:
      break;
    case TS::MIV_Main:
    case TS::MIV_Geometry_Absent:
      PTL_CHECK(mivSpec, "Table A-1", ai_attribute_type_id == ATI::ATTR_TEXTURE);
      PTL_CHECK(mivSpec, "Table A-1", !ai_attribute_msb_align_flag);

      if (ai_attribute_type_id == ATI::ATTR_TEXTURE) {
        PTL_CHECK(mivSpec, "Table A-1", ai_attribute_dimension_minus1 == 2);
      }
      break;
    case TS::MIV_Extended:
      PTL_CHECK(mivSpec, "Table A-1",
                ai_attribute_type_id == ATI::ATTR_TEXTURE ||
                    ai_attribute_type_id == ATI::ATTR_TRANSPARENCY);
      PTL_CHECK(mivSpec, "Table A-1", !ai_attribute_msb_align_flag);

      if (ai_attribute_type_id == ATI::ATTR_TEXTURE) {
        PTL_CHECK(mivSpec, "Table A-1", ai_attribute_dimension_minus1 == 2);
      } else if (ai_attribute_type_id == ATI::ATTR_TRANSPARENCY) {
        PTL_CHECK(mivSpec, "Table A-1", ai_attribute_dimension_minus1 == 0);
      }
      break;
    }
  }

  void checkVpsMivExtension(const MivBitstream::VpsMivExtension &vme) const {
    const auto vme_embedded_occupancy_enabled_flag = vme.vme_embedded_occupancy_enabled_flag();

    switch (ptl_profile_toolset_idc()) {
    case TS::VPCC_Basic:
    case TS::VPCC_Extended:
      break;
    case TS::MIV_Main:
      PTL_CHECK(mivSpec, "Table A-1", vme_embedded_occupancy_enabled_flag);
      break;
    case TS::MIV_Extended:
      if (ptc_restricted_geometry_flag()) {
        PTL_CHECK(mivSpec, "Table A-1", !vme_embedded_occupancy_enabled_flag);
      }
      break;
    case TS::MIV_Geometry_Absent:
      PTL_CHECK(mivSpec, "Table A-1", !vme_embedded_occupancy_enabled_flag);
      break;
    }
  }

  void activateCasps(const MivBitstream::CommonAtlasSequenceParameterSetRBSP &casps) {
    if (casps.casps_miv_extension_present_flag()) {
      const auto &casme = casps.casps_miv_extension();

      if (casme.casme_vui_params_present_flag()) {
        const auto &vui = casme.vui_parameters();

        if (vui.vui_timing_info_present_flag()) {
          m_frameRate = static_cast<double>(vui.vui_time_scale()) /
                        static_cast<double>(vui.vui_num_units_in_tick());
        }
      }
    }
  }

  void checkAsps(MivBitstream::AtlasId atlasId,
                 const MivBitstream::AtlasSequenceParameterSetRBSP &asps) {
    PRECONDITION(m_vps.has_value());

    switch (ptl_profile_toolset_idc()) {
    case TS::VPCC_Basic:
      PTL_CHECK(v3cSpec, "Table H-3", !asps.asps_eom_patch_enabled_flag());
      PTL_CHECK(v3cSpec, "Table H-3", !asps.asps_plr_enabled_flag());
      PTL_CHECK(v3cSpec, "Table H-3", !asps.asps_use_eight_orientations_flag());
      PTL_CHECK(v3cSpec, "Table H-3", !asps.asps_extended_projection_enabled_flag());
      PTL_CHECK(v3cSpec, "Table H-3", !asps.asps_miv_extension_present_flag());
      break;
    case TS::VPCC_Extended:
      PTL_CHECK(v3cSpec, "Table H-3", !asps.asps_miv_extension_present_flag());
      break;
    case TS::MIV_Main:
    case TS::MIV_Extended:
    case TS::MIV_Geometry_Absent:
      PTL_CHECK(mivSpec, "Table A-1", !asps.asps_max_dec_atlas_frame_buffering_minus1());
      PTL_CHECK(mivSpec, "Table A-1", !asps.asps_long_term_ref_atlas_frames_flag());
      PTL_CHECK(mivSpec, "Table A-1", !asps.asps_pixel_deinterleaving_enabled_flag());
      PTL_CHECK(mivSpec, "Table A-1", !asps.asps_patch_precedence_order_flag());
      PTL_CHECK(mivSpec, "Table A-1", !asps.asps_raw_patch_enabled_flag());
      PTL_CHECK(mivSpec, "Table A-1", !asps.asps_eom_patch_enabled_flag());
      PTL_CHECK(mivSpec, "Table A-1", !asps.asps_plr_enabled_flag());
      PTL_CHECK(mivSpec, "Table A-1", !asps.asps_vpcc_extension_present_flag());
      break;
    }

    if (ptc_restricted_geometry_flag()) {
      // Because of asme_patch_constant_depth_flag, the ASME is required
      PTL_CHECK(mivSpec, "Table A-1 (implicit)", asps.asps_miv_extension_present_flag());
    }

    if (asps.asps_miv_extension_present_flag()) {
      checkAsme(atlasId, asps.asps_miv_extension());
    }

    levelCheck(v3cSpec, "Table A-6", "MaxAtlasSize",
               asps.asps_frame_width() * asps.asps_frame_height(),
               levelLimit(2'228'224, 8'912'896, 35'651'584, 134'217'728));
  }

  void checkAsme(MivBitstream::AtlasId atlasId, const MivBitstream::AspsMivExtension &asme) const {
    const auto vps_geometry_video_present_flag = m_vps->vps_geometry_video_present_flag(atlasId);
    const auto asme_patch_constant_depth_flag = asme.asme_patch_constant_depth_flag();
    const auto pin_geometry_present_flag =
        m_vps->vps_packed_video_present_flag(atlasId)
            ? m_vps->packing_information(atlasId).pin_geometry_present_flag()
            : false;

    switch (ptl_profile_toolset_idc()) {
    case TS::MIV_Main:
      PTL_CHECK(mivSpec, "Table A-1", !asme_patch_constant_depth_flag);
      break;
    case TS::MIV_Extended:
      // NOTE(MPEG/MIV/Specs/23090-12#436): MIV ext. profile with packed geometry is out-of-profile
      PTL_CHECK(mivSpec, "Table A-1",
                vps_geometry_video_present_flag || pin_geometry_present_flag ||
                    asme_patch_constant_depth_flag);
      break;
    case TS::VPCC_Basic:
    case TS::VPCC_Extended:
    case TS::MIV_Geometry_Absent:
      break;
    }
  }

  void checkAfps(const MivBitstream::AtlasFrameParameterSetRBSP &afps) const {
    PTL_CHECK(mivSpec, "Table A-1", !afps.afps_lod_mode_enabled_flag());
    PTL_CHECK(mivSpec, "Table A-1", !afps.afps_raw_3d_offset_bit_count_explicit_mode_flag());
  }

  void checkAtl(const MivBitstream::NalUnitHeader &nuh,
                const MivBitstream::AtlasTileLayerRBSP &atl) const {
    const auto &ath = atl.atlas_tile_header();
    const auto ath_type = ath.ath_type();
    const auto &atdu = atl.atlas_tile_data_unit();

    switch (ptl_profile_toolset_idc()) {
    case TS::VPCC_Basic:
    case TS::VPCC_Extended:
      break;
    case TS::MIV_Main:
    case TS::MIV_Extended:
    case TS::MIV_Geometry_Absent:
      PTL_CHECK(mivSpec, "Table A-1", ath_type == MivBitstream::AthType::I_TILE);

      for (size_t p = 0; p < atdu.atduTotalNumberOfPatches(); ++p) {
        PTL_CHECK(mivSpec, "Table A-1", atdu.atdu_patch_mode(p) == APM::I_INTRA);
      }
      break;
    }

    // NOTE(BK): Name idrNuts to work around what appears to be a GCC 12 false alarm:
    // error: dangling pointer to an unnamed temporary may be used [-Werror=dangling-pointer=]
    static constexpr auto idrNuts = std::array{NUT::NAL_IDR_W_RADL, NUT::NAL_IDR_N_LP,
                                               NUT::NAL_GIDR_W_RADL, NUT::NAL_GIDR_N_LP};
    const auto idrCodedAtlas = contains(idrNuts, nuh.nal_unit_type());
    PTL_CHECK(v3cSpec, "A.6.1", !idrCodedAtlas || ath.ath_atlas_frm_order_cnt_lsb() == 0);
  }

  void checkCaf(const MivBitstream::NalUnitHeader &nuh,
                const MivBitstream::CommonAtlasFrameRBSP &caf) const {
    const auto irapCodedCommonAtlas = nuh.nal_unit_type() == NUT::NAL_CAF_IDR;
    PTL_CHECK(v3cSpec, "A.6.1",
              !irapCodedCommonAtlas || caf.caf_common_atlas_frm_order_cnt_lsb() == 0);
  }

  void checkVideoFrame(VUT vut, const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
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
    PTL_CHECK(v3cSpec, "Table A-2",
              colorFormat == CF::YUV420 ||
                  (colorFormat == CF::YUV400 && (ptl_profile_codec_group_idc == CG::HEVC444 ||
                                                 ptl_profile_codec_group_idc == CG::VVC_Main10)) ||
                  (colorFormat == CF::YUV444 && vut == VUT::V3C_AVD &&
                   ptl_profile_codec_group_idc == CG::HEVC444));

    const auto bitDepth = frame.getBitDepth();
    PTL_CHECK(v3cSpec, "Table A-2",
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
      // NOTE(MPEG/PCC/Specs/23090-5#496): Table A-2 does not have columns for packed video
      break;
    default:
      UNREACHABLE;
    }
  }

  void checkOccupancyVideoFrame(const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                                const Common::Frame<> &frame) const {
    int32_t occScaleX = 1;
    int32_t occScaleY = 1;

    if (m_vps->vpsMivExtensionPresentFlag() &&
        m_vps->vps_miv_extension().vme_occupancy_scale_enabled_flag()) {
      const auto &asme = asps.asps_miv_extension();

      occScaleX = asme.asme_occupancy_scale_factor_x_minus1() + 1;
      occScaleY = asme.asme_occupancy_scale_factor_y_minus1() + 1;
    }

    const auto asmeOccupancyFrameWidth = asps.asps_frame_width() / occScaleX;
    const auto asmeOccupancyFrameHeight = asps.asps_frame_height() / occScaleY;

    PTL_CHECK(mivSpec, "A.4.1", frame.getWidth() == asmeOccupancyFrameWidth);
    PTL_CHECK(mivSpec, "A.4.1", frame.getHeight() == asmeOccupancyFrameHeight);
  }

  void checkGeometryVideoFrame(const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                               const Common::Frame<> &frame) const {
    int32_t geoScaleX = 1;
    int32_t geoScaleY = 1;

    if (m_vps->vpsMivExtensionPresentFlag() &&
        m_vps->vps_miv_extension().vme_geometry_scale_enabled_flag()) {
      const auto &asme = asps.asps_miv_extension();

      geoScaleX = asme.asme_geometry_scale_factor_x_minus1() + 1;
      geoScaleY = asme.asme_geometry_scale_factor_y_minus1() + 1;
    }

    const auto asmeGeometryFrameWidth = asps.asps_frame_width() / geoScaleX;
    const auto asmeGeometryFrameHeight = asps.asps_frame_height() / geoScaleY;

    PTL_CHECK(mivSpec, "A.4.1", frame.getWidth() == asmeGeometryFrameWidth);
    PTL_CHECK(mivSpec, "A.4.1", frame.getHeight() == asmeGeometryFrameHeight);
  }

  void checkAttributeVideoFrame(const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                                const Common::Frame<> &frame) const {
    const auto aspsFrameWidth = asps.asps_frame_width();
    const auto aspsFrameHeight = asps.asps_frame_height();

    PTL_CHECK(mivSpec, "A.4.1", frame.getWidth() == aspsFrameWidth);
    PTL_CHECK(mivSpec, "A.4.1", frame.getHeight() == aspsFrameHeight);
  }

  [[nodiscard]] auto checkNumProjPatches(const MivBitstream::AccessUnit &frame) const {
    return std::accumulate(frame.atlas.cbegin(), frame.atlas.cend(), int64_t{},
                           [this](int64_t init, const auto &atlas) {
                             return init +
                                    levelCheck(v3cSpec, "Table A-6", "MaxNumProjPatches",
                                               downCast<int64_t>(atlas.patchParamsList.size()),
                                               levelLimit(2'048, 4'096, 16'384, 32'384, 65'536,
                                                          65'536, 262'140, 262'140));
                           });
  }

  [[nodiscard]] auto checkNumLumaSamples(const MivBitstream::AccessUnit &frame) const {
    auto result = int64_t{};

    const auto count = [&result, this](const auto &frame_) {
      if (!frame_.empty()) {
        result += levelCheck(v3cSpec, "Table A-7", "MaxPictureSize",
                             frame_.getWidth() * frame_.getHeight(),
                             levelLimit(2'228'224, 8'912'896, 35'651'584, 142'606'336));
      }
    };

    for (const auto &atlas : frame.atlas) {
      count(atlas.decOccFrame);
      count(atlas.decGeoFrame);

      for (const auto &decAttrFrame : atlas.decAttrFrame) {
        count(decAttrFrame);
      }

      count(atlas.decPckFrame);
    }

    return result;
  }

  template <typename Integer>
  [[nodiscard]] auto maxPerSec(Integer V3cFrameVariables::*field) const {
    return std::accumulate(
        m_window.cbegin(), m_window.cend(), Integer{},
        [field](Integer init, const V3cFrameVariables &frame) { return init + frame.*field; });
  }

  void checkV3cFrame([[maybe_unused]] const MivBitstream::AccessUnit &frame) {
    VERIFY(m_vps);
    PTL_CHECK(mivSpec, "A.4.3", !ptc_one_v3c_frame_only_flag() || !m_haveV3cFrame);
    m_haveV3cFrame = true;

    [[maybe_unused]] auto &variables = m_window.emplace_back();

    // NOTE(MPEG/PCC/Specs/23090-5#550): Annex A normatively references informative Annex B
    // NOTE(MPEG/PCC/Specs/23090-5#569): It is unclear how to aggregate over atlases

    variables.numProjPatches = checkNumProjPatches(frame);
    variables.numLumaSamples = checkNumLumaSamples(frame);

    while (m_frameRate < static_cast<double>(m_window.size())) {
      m_window.erase(m_window.begin());
    }

    levelCheck(v3cSpec, "Table A-6", "MaxProjPatchesPerSec",
               maxPerSec(&V3cFrameVariables::numProjPatches),
               levelLimit(65'536, 131'072, 524'288, 1'036'288, 2'097'152, 4'194'304, 8'388'608,
                          16'777'216));

    // NOTE(MPEG/PCC/Specs/23090-5#570): Table A-6: Max CAB size and MaxAtlasBR limits are
    // unpractically high. Not implementing any test.

    levelCheck(v3cSpec, "Table A-7", "MaxAggregateLumaSr",
               maxPerSec(&V3cFrameVariables::numLumaSamples),
               levelLimit(133'693'440, 267'386'880, 534'773'760, 1'069'547'520, 2'139'095'040,
                          4'278'190'080, 8'556'380'160, 17'112'760'320));
  }

  [[nodiscard]] auto frameRate() const { return m_frameRate; }
};

PtlChecker::PtlChecker() : m_impl{new Impl} {}

PtlChecker::PtlChecker(PtlChecker &&) noexcept = default;

auto PtlChecker::operator=(PtlChecker &&) noexcept -> PtlChecker & = default;

PtlChecker::~PtlChecker() = default;

void PtlChecker::replaceLogger(Logger value) { m_impl->replaceLogger(std::move(value)); }

void PtlChecker::checkVuh(const MivBitstream::V3cUnitHeader &vuh) { m_impl->checkVuh(vuh); }

void PtlChecker::checkNuh(const MivBitstream::NalUnitHeader &nuh) { m_impl->checkNuh(nuh); }

void PtlChecker::checkAndActivateVps(const MivBitstream::V3cParameterSet &vps) {
  m_impl->checkAndActivateVps(vps);
}

void PtlChecker::activateCasps(const MivBitstream::CommonAtlasSequenceParameterSetRBSP &casps) {
  m_impl->activateCasps(casps);
}

void PtlChecker::checkAsps(MivBitstream::AtlasId atlasId,
                           const MivBitstream::AtlasSequenceParameterSetRBSP &asps) {
  m_impl->checkAsps(atlasId, asps);
}

void PtlChecker::checkAfps(const MivBitstream::AtlasFrameParameterSetRBSP &afps) {
  m_impl->checkAfps(afps);
}

void PtlChecker::checkAtl(const MivBitstream::NalUnitHeader &nuh,
                          const MivBitstream::AtlasTileLayerRBSP &atl) {
  m_impl->checkAtl(nuh, atl);
}

void PtlChecker::checkCaf(const MivBitstream::NalUnitHeader &nuh,
                          const MivBitstream::CommonAtlasFrameRBSP &caf) {
  m_impl->checkCaf(nuh, caf);
}

void PtlChecker::checkVideoFrame(MivBitstream::VuhUnitType vut,
                                 const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                                 const Common::Frame<> &frame) {
  m_impl->checkVideoFrame(vut, asps, frame);
}

void PtlChecker::checkV3cFrame(const MivBitstream::AccessUnit &frame) {
  m_impl->checkV3cFrame(frame);
}

auto PtlChecker::frameRate() const -> double { return m_impl->frameRate(); }
} // namespace TMIV::PtlChecker
