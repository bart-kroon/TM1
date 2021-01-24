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

#include <catch2/catch.hpp>

#include <TMIV/MivBitstream/PatchParamsList.h>

using TMIV::Common::Vec2i;
using TMIV::Common::Vec3i;
using TMIV::MivBitstream::AtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::AtlasTileHeader;
using TMIV::MivBitstream::FlexiblePatchOrientation;
using TMIV::MivBitstream::PatchDataUnit;
using TMIV::MivBitstream::PatchParams;

TEST_CASE("TMIV::MivBitstream::PatchParams") {
  SECTION("Decode patch data unit") {
    SECTION("Minimal example") {
      const auto unit = PatchParams::decodePdu({}, {}, {});

      REQUIRE(unit.atlasPatch2dPosX() == 0);
      REQUIRE(unit.atlasPatch2dPosY() == 0);
      REQUIRE(unit.atlasPatch2dSizeX() == 1);
      REQUIRE(unit.atlasPatch2dSizeY() == 1);
      REQUIRE(unit.atlasPatch3dOffsetU() == 0);
      REQUIRE(unit.atlasPatch3dOffsetV() == 0);
      REQUIRE(unit.atlasPatch3dOffsetD() == 0);
      REQUIRE(unit.atlasPatch3dRangeD() == 1);
      REQUIRE(unit.atlasPatchProjectionId() == 0);
      REQUIRE(unit.atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_NULL);
      REQUIRE(unit.atlasPatchEntityId() == std::nullopt);
      REQUIRE(unit.atlasPatchDepthOccMapThreshold() == std::nullopt);
      REQUIRE_THROWS(unit.atlasPatchAttributeOffset());
    }

    SECTION("Typical example") {
      auto pdu = PatchDataUnit{};
      pdu.pdu_2d_pos_x(1)
          .pdu_2d_pos_y(2)
          .pdu_2d_size_x_minus1(3)
          .pdu_2d_size_y_minus1(4)
          .pdu_3d_offset_u(5)
          .pdu_3d_offset_v(6)
          .pdu_projection_id(7)
          .pdu_orientation_index(FlexiblePatchOrientation::FPO_ROT270);

      auto asps = AtlasSequenceParameterSetRBSP{};
      asps.asps_log2_patch_packing_block_size(3)
          .asps_geometry_2d_bit_depth_minus1(4)
          .asps_geometry_3d_bit_depth_minus1(5);

      const auto unit = PatchParams::decodePdu(pdu, asps, {});

      REQUIRE(unit.atlasPatch2dPosX() == 8);
      REQUIRE(unit.atlasPatch2dPosY() == 16);
      REQUIRE(unit.atlasPatch2dSizeX() == 32);
      REQUIRE(unit.atlasPatch2dSizeY() == 40);
      REQUIRE(unit.atlasPatch3dOffsetU() == 5);
      REQUIRE(unit.atlasPatch3dOffsetV() == 6);
      REQUIRE(unit.atlasPatch3dOffsetD() == 0);
      REQUIRE(unit.atlasPatch3dRangeD() == 31);
      REQUIRE(unit.atlasPatchProjectionId() == 7);
      REQUIRE(unit.atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_ROT270);
      REQUIRE(unit.atlasPatchEntityId() == std::nullopt);
      REQUIRE(unit.atlasPatchDepthOccMapThreshold() == std::nullopt);
      REQUIRE_THROWS(unit.atlasPatchAttributeOffset());
    }

    SECTION("Elaborate example") {
      auto pdu = PatchDataUnit{};
      pdu.pdu_2d_pos_x(1)
          .pdu_2d_pos_y(2)
          .pdu_2d_size_x_minus1(3)
          .pdu_2d_size_y_minus1(4)
          .pdu_3d_offset_u(5)
          .pdu_3d_offset_v(6)
          .pdu_3d_range_d(7)
          .pdu_projection_id(8)
          .pdu_orientation_index(FlexiblePatchOrientation::FPO_ROT270)
          .pdu_miv_extension()
          .pdu_depth_occ_threshold(9)
          .pdu_attribute_offset(Vec3i{10, 11, 12})
          .pdu_entity_id(13);

      auto asps = AtlasSequenceParameterSetRBSP{};
      asps.asps_log2_patch_packing_block_size(3)
          .asps_geometry_2d_bit_depth_minus1(4)
          .asps_geometry_3d_bit_depth_minus1(5)
          .asps_normal_axis_max_delta_value_enabled_flag(true)
          .asps_patch_size_quantizer_present_flag(true)
          .asps_extension_present_flag(true)
          .asps_miv_extension_present_flag(true)
          .asps_miv_extension()
          .asme_embedded_occupancy_enabled_flag(true)
          .asme_depth_occ_threshold_flag(true)
          .asme_patch_attribute_offset_enabled_flag(true);

      auto ath = AtlasTileHeader();
      ath.ath_pos_min_d_quantizer(6)
          .ath_pos_delta_max_d_quantizer(7)
          .ath_patch_size_x_info_quantizer(2)
          .ath_patch_size_y_info_quantizer(1);

      const auto unit = PatchParams::decodePdu(pdu, asps, ath);

      REQUIRE(unit.atlasPatch2dPosX() == 8);
      REQUIRE(unit.atlasPatch2dPosY() == 16);
      REQUIRE(unit.atlasPatch2dSizeX() == 16);
      REQUIRE(unit.atlasPatch2dSizeY() == 10);
      REQUIRE(unit.atlasPatch3dOffsetU() == 5);
      REQUIRE(unit.atlasPatch3dOffsetV() == 6);
      REQUIRE(unit.atlasPatch3dOffsetD() == 0);
      REQUIRE(unit.atlasPatch3dRangeD() == 895);
      REQUIRE(unit.atlasPatchProjectionId() == 8);
      REQUIRE(unit.atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_ROT270);
      REQUIRE(unit.atlasPatchEntityId() == 13);
      REQUIRE(unit.atlasPatchDepthOccMapThreshold() == 9);
      REQUIRE(unit.atlasPatchAttributeOffset() == Vec3i{10, 11, 12});
    }
  }

  SECTION("Encode patch data unit") {
    SECTION("Minimal example") {
      auto unit = PatchParams{};
      unit.atlasPatch2dSizeX(1).atlasPatch2dSizeY(1).atlasPatchOrientationIndex(
          FlexiblePatchOrientation::FPO_NULL);

      const auto pdu = unit.encodePdu({}, {});

      REQUIRE(pdu.pdu_2d_pos_x() == 0);
      REQUIRE(pdu.pdu_2d_pos_y() == 0);
      REQUIRE(pdu.pdu_2d_size_x_minus1() == 0);
      REQUIRE(pdu.pdu_2d_size_y_minus1() == 0);
      REQUIRE(pdu.pdu_3d_offset_u() == 0);
      REQUIRE(pdu.pdu_3d_offset_v() == 0);
      REQUIRE(pdu.pdu_3d_offset_d() == 0);
      REQUIRE_THROWS(pdu.pdu_3d_range_d());
      REQUIRE(pdu.pdu_projection_id() == 0);
      REQUIRE(pdu.pdu_orientation_index() == FlexiblePatchOrientation::FPO_NULL);
      REQUIRE(!pdu.pdu_lod_enabled_flag());
      REQUIRE(pdu.pdu_lod_scale_x_minus1() == 0);
      REQUIRE(pdu.pdu_lod_scale_y_idc() == 0);
      REQUIRE_THROWS(pdu.pdu_miv_extension().pdu_attribute_offset());
      REQUIRE(pdu.pdu_miv_extension().pdu_entity_id() == 0);
      REQUIRE_THROWS(pdu.pdu_miv_extension().pdu_depth_occ_threshold());
    }

    SECTION("Typical example") {
      auto asps = AtlasSequenceParameterSetRBSP{};
      asps.asps_log2_patch_packing_block_size(3)
          .asps_geometry_2d_bit_depth_minus1(4)
          .asps_geometry_3d_bit_depth_minus1(5);

      auto unit = PatchParams{};
      unit.atlasPatch2dPosX(8)
          .atlasPatch2dPosY(16)
          .atlasPatch2dSizeX(32)
          .atlasPatch2dSizeY(40)
          .atlasPatch3dOffsetU(5)
          .atlasPatch3dOffsetV(6)
          .atlasPatch3dRangeD(31)
          .atlasPatchProjectionId(7)
          .atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_ROT270);

      const auto pdu = unit.encodePdu(asps, {});

      REQUIRE(pdu.pdu_2d_pos_x() == 1);
      REQUIRE(pdu.pdu_2d_pos_y() == 2);
      REQUIRE(pdu.pdu_2d_size_x_minus1() == 3);
      REQUIRE(pdu.pdu_2d_size_y_minus1() == 4);
      REQUIRE(pdu.pdu_3d_offset_u() == 5);
      REQUIRE(pdu.pdu_3d_offset_v() == 6);
      REQUIRE(pdu.pdu_projection_id() == 7);
      REQUIRE(pdu.pdu_orientation_index() == FlexiblePatchOrientation::FPO_ROT270);
    }

    SECTION("Elaborate example") {
      auto unit = PatchParams{};
      unit.atlasPatch2dPosX(8)
          .atlasPatch2dPosY(16)
          .atlasPatch2dSizeX(16)
          .atlasPatch2dSizeY(10)
          .atlasPatch3dOffsetU(5)
          .atlasPatch3dOffsetV(6)
          .atlasPatch3dOffsetD(0)
          .atlasPatch3dRangeD(895)
          .atlasPatchProjectionId(8)
          .atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_ROT270)
          .atlasPatchEntityId(13)
          .atlasPatchDepthOccMapThreshold(9)
          .atlasPatchAttributeOffset(Vec3i{10, 11, 12});

      auto asps = AtlasSequenceParameterSetRBSP{};
      asps.asps_log2_patch_packing_block_size(3)
          .asps_geometry_2d_bit_depth_minus1(4)
          .asps_geometry_3d_bit_depth_minus1(5)
          .asps_normal_axis_max_delta_value_enabled_flag(true)
          .asps_patch_size_quantizer_present_flag(true)
          .asps_extension_present_flag(true)
          .asps_miv_extension_present_flag(true)
          .asps_miv_extension()
          .asme_embedded_occupancy_enabled_flag(true)
          .asme_depth_occ_threshold_flag(true)
          .asme_patch_attribute_offset_enabled_flag(true);

      auto ath = AtlasTileHeader();
      ath.ath_pos_min_d_quantizer(6)
          .ath_pos_delta_max_d_quantizer(7)
          .ath_patch_size_x_info_quantizer(2)
          .ath_patch_size_y_info_quantizer(1);

      const auto pdu = unit.encodePdu(asps, ath);

      REQUIRE(pdu.pdu_2d_pos_x() == 1);
      REQUIRE(pdu.pdu_2d_pos_y() == 2);
      REQUIRE(pdu.pdu_2d_size_x_minus1() == 3);
      REQUIRE(pdu.pdu_2d_size_y_minus1() == 4);
      REQUIRE(pdu.pdu_3d_offset_u() == 5);
      REQUIRE(pdu.pdu_3d_offset_v() == 6);
      REQUIRE(pdu.pdu_3d_range_d() == 7);
      REQUIRE(pdu.pdu_projection_id() == 8);
      REQUIRE(pdu.pdu_orientation_index() == FlexiblePatchOrientation::FPO_ROT270);
      REQUIRE(pdu.pdu_miv_extension().pdu_depth_occ_threshold() == 9);
      REQUIRE(pdu.pdu_miv_extension().pdu_attribute_offset() == Vec3i{10, 11, 12});
      REQUIRE(pdu.pdu_miv_extension().pdu_entity_id() == 13);
    }
  }

  SECTION("View-atlas coordinate transformations") {
    auto unit = PatchParams{};
    unit.atlasPatch2dPosX(8)
        .atlasPatch2dPosY(16)
        .atlasPatch2dSizeX(16)
        .atlasPatch2dSizeY(10)
        .atlasPatch3dOffsetU(5)
        .atlasPatch3dOffsetV(6);

    SECTION("View to atlas transformation") {
      SECTION("FPO_NULL") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_NULL);
        REQUIRE(unit.viewToAtlas(Vec2i{0, 0}) == Vec2i{3, 10});
        REQUIRE(unit.viewToAtlas(Vec2i{0, 1}) == Vec2i{3, 11});
        REQUIRE(unit.viewToAtlas(Vec2i{1, 0}) == Vec2i{4, 10});
      }

      SECTION("FPO_SWAP") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_SWAP);
        REQUIRE(unit.viewToAtlas(Vec2i{0, 0}) == Vec2i{2, 11});
        REQUIRE(unit.viewToAtlas(Vec2i{0, 1}) == Vec2i{3, 11});
        REQUIRE(unit.viewToAtlas(Vec2i{1, 0}) == Vec2i{2, 12});
      }

      SECTION("FPO_ROT90") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_ROT90);
        REQUIRE(unit.viewToAtlas(Vec2i{0, 0}) == Vec2i{29, 11});
        REQUIRE(unit.viewToAtlas(Vec2i{0, 1}) == Vec2i{28, 11});
        REQUIRE(unit.viewToAtlas(Vec2i{1, 0}) == Vec2i{29, 12});
      }

      SECTION("FPO_ROT180") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_ROT180);
        REQUIRE(unit.viewToAtlas(Vec2i{0, 0}) == Vec2i{28, 31});
        REQUIRE(unit.viewToAtlas(Vec2i{0, 1}) == Vec2i{28, 30});
        REQUIRE(unit.viewToAtlas(Vec2i{1, 0}) == Vec2i{27, 31});
      }

      SECTION("FPO_ROT270") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_ROT270);
        REQUIRE(unit.viewToAtlas(Vec2i{0, 0}) == Vec2i{2, 30});
        REQUIRE(unit.viewToAtlas(Vec2i{0, 1}) == Vec2i{3, 30});
        REQUIRE(unit.viewToAtlas(Vec2i{1, 0}) == Vec2i{2, 29});
      }

      SECTION("FPO_MIRROR") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_MIRROR);
        REQUIRE(unit.viewToAtlas(Vec2i{0, 0}) == Vec2i{28, 10});
        REQUIRE(unit.viewToAtlas(Vec2i{0, 1}) == Vec2i{28, 11});
        REQUIRE(unit.viewToAtlas(Vec2i{1, 0}) == Vec2i{27, 10});
      }

      SECTION("FPO_MROT90") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_MROT90);
        REQUIRE(unit.viewToAtlas(Vec2i{0, 0}) == Vec2i{29, 30});
        REQUIRE(unit.viewToAtlas(Vec2i{0, 1}) == Vec2i{28, 30});
        REQUIRE(unit.viewToAtlas(Vec2i{1, 0}) == Vec2i{29, 29});
      }

      SECTION("FPO_MROT180") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_MROT180);
        REQUIRE(unit.viewToAtlas(Vec2i{0, 0}) == Vec2i{3, 31});
        REQUIRE(unit.viewToAtlas(Vec2i{0, 1}) == Vec2i{3, 30});
        REQUIRE(unit.viewToAtlas(Vec2i{1, 0}) == Vec2i{4, 31});
      }
    }

    SECTION("Atlas to view transformation") {
      SECTION("FPO_NULL") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_NULL);
        REQUIRE(unit.atlasToView(Vec2i{3, 10}) == Vec2i{0, 0});
        REQUIRE(unit.atlasToView(Vec2i{3, 11}) == Vec2i{0, 1});
        REQUIRE(unit.atlasToView(Vec2i{4, 10}) == Vec2i{1, 0});
      }

      SECTION("FPO_SWAP") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_SWAP);
        REQUIRE(unit.atlasToView(Vec2i{2, 11}) == Vec2i{0, 0});
        REQUIRE(unit.atlasToView(Vec2i{3, 11}) == Vec2i{0, 1});
        REQUIRE(unit.atlasToView(Vec2i{2, 12}) == Vec2i{1, 0});
      }

      SECTION("FPO_ROT90") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_ROT90);
        REQUIRE(unit.atlasToView(Vec2i{29, 11}) == Vec2i{0, 0});
        REQUIRE(unit.atlasToView(Vec2i{28, 11}) == Vec2i{0, 1});
        REQUIRE(unit.atlasToView(Vec2i{29, 12}) == Vec2i{1, 0});
      }

      SECTION("FPO_ROT180") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_ROT180);
        REQUIRE(unit.atlasToView(Vec2i{28, 31}) == Vec2i{0, 0});
        REQUIRE(unit.atlasToView(Vec2i{28, 30}) == Vec2i{0, 1});
        REQUIRE(unit.atlasToView(Vec2i{27, 31}) == Vec2i{1, 0});
      }

      SECTION("FPO_ROT270") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_ROT270);
        REQUIRE(unit.atlasToView(Vec2i{2, 30}) == Vec2i{0, 0});
        REQUIRE(unit.atlasToView(Vec2i{3, 30}) == Vec2i{0, 1});
        REQUIRE(unit.atlasToView(Vec2i{2, 29}) == Vec2i{1, 0});
      }

      SECTION("FPO_MIRROR") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_MIRROR);
        REQUIRE(unit.atlasToView(Vec2i{28, 10}) == Vec2i{0, 0});
        REQUIRE(unit.atlasToView(Vec2i{28, 11}) == Vec2i{0, 1});
        REQUIRE(unit.atlasToView(Vec2i{27, 10}) == Vec2i{1, 0});
      }

      SECTION("FPO_MROT90") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_MROT90);
        REQUIRE(unit.atlasToView(Vec2i{29, 30}) == Vec2i{0, 0});
        REQUIRE(unit.atlasToView(Vec2i{28, 30}) == Vec2i{0, 1});
        REQUIRE(unit.atlasToView(Vec2i{29, 29}) == Vec2i{1, 0});
      }

      SECTION("FPO_MROT180") {
        unit.atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_MROT180);
        REQUIRE(unit.atlasToView(Vec2i{3, 31}) == Vec2i{0, 0});
        REQUIRE(unit.atlasToView(Vec2i{3, 30}) == Vec2i{0, 1});
        REQUIRE(unit.atlasToView(Vec2i{4, 31}) == Vec2i{1, 0});
      }
    }
  }
}
