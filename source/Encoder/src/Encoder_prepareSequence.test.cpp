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

#include <catch2/catch.hpp>

#include <TMIV/Encoder/Encoder.h>

using namespace std::string_view_literals;

namespace test {
auto configuration1() {
  return TMIV::Encoder::Configuration{TMIV::Common::Json::parse(R"({
    "intraPeriod": 1,
    "blockSizeDepthQualityDependent": [2, 4],
    "haveTextureVideo": false,
    "haveGeometryVideo": true,
    "bitDepthGeometryVideo": 10,
    "haveOccupancyVideo": false,
    "framePacking": false,
    "oneViewPerAtlasFlag": false,
    "dynamicDepthRange": false,
    "halveDepthRange": true,
    "randomAccess": false,
    "patchRedundancyRemoval": true,
    "viewportCameraParametersSei": false,
    "viewportPositionSei": false,
    "numGroups": 2,
    "maxEntityId": 0,
    "maxLumaSampleRate": 1000000000,
    "maxLumaPictureSize": 8000000,
    "maxAtlases": 2,
    "codecGroupIdc": "HEVC Main10",
    "toolsetIdc": "MIV Main",
    "reconstructionIdc": "Rec Unconstrained",
    "levelIdc": "3.5",
    "depthOccThresholdIfSet": 0.0625,
    "nonAggregatedMaskDilationIter": 0
})"sv)};
}

const auto sequenceConfig1 = TMIV::MivBitstream::SequenceConfig{TMIV::Common::Json::parse(R"(
{
    "Version": "4.0",
    "BoundingBox_center": [ -0.5, -0.5, 1.0 ],
    "Content_name": "Chess",
    "Fps": 30,
    "Frames_number": 300,
    "lengthsInMeters": false,
    "Informative": {
        "Cameras_number": 10,
        "RigRadius": 0.3,
        "Document": "MPEG128/m50787"
    },
    "cameras": [ {
        "Background": 1,
        "BitDepthColor": 10,
        "BitDepthDepth": 16,
        "ColorSpace": "YUV420",
        "DepthColorSpace": "YUV420",
        "Depth_range": [0.1, 500],
        "Depthmap": 1,
        "Name": "viewport",
        "Projection": "Perspective",
        "Position": [ -0.5, -0.5, 1.0 ],
        "Resolution": [2048, 2048],
        "Rotation": [ 0.0, 0.0, 0.0 ],
        "Focal": [1024, 1024],
        "Principle_point": [1024, 1024]
    },  {
        "BitDepthColor": 10,
        "BitDepthDepth": 16,
        "BitDepthTransparency": 13,
        "BitDepthEntitities": 15,
        "ColorSpace": "YUV420",
        "DepthColorSpace": "YUV420",
        "TransparencyColorSpace": "YUV420",
        "EntitiesColorSpace": "YUV420",
        "Depth_range": [ 0.1, 500.0 ],
        "Hor_range": [ -90.0, 90.0 ],
        "HasInvalidDepth": false,
        "Name": "v0",
        "Position": [ -0.5, -0.5, 1.2999999523162842 ],
        "Projection": "Equirectangular",
        "Resolution": [ 2048, 2048 ],
        "Rotation": [ -90.00000933466734, -90.0000161648565, 0.0 ],
        "Ver_range": [ -90.0, 90.0 ]
    },  {
        "BitDepthColor": 10,
        "BitDepthDepth": 16,
        "BitDepthTransparency": 13,
        "BitDepthEntitities": 15,
        "ColorSpace": "YUV420",
        "DepthColorSpace": "YUV420",
        "TransparencyColorSpace": "YUV420",
        "EntitiesColorSpace": "YUV420",
        "Depth_range": [ 0.1, 500.0 ],
        "Hor_range": [ -90.0, 90.0 ],
        "HasInvalidDepth": false,
        "Name": "v5",
        "Position": [ -0.5, -0.5, 1.2999999523162842 ],
        "Projection": "Equirectangular",
        "Resolution": [ 2048, 2048 ],
        "Rotation": [ -90.00000933466734, -90.0000161648565, 0.0 ],
        "Ver_range": [ -90.0, 90.0 ]
    } ]
}
)")};

class FakeDepthQualityAssessor final : public TMIV::DepthQualityAssessor::IDepthQualityAssessor {
public:
  FakeDepthQualityAssessor() = default;
  FakeDepthQualityAssessor(TMIV::MivBitstream::ViewParamsList viewParamsList,
                           const TMIV::Common::DeepFrameList &frame, std::vector<bool> results)
      : m_viewParamsList{std::move(viewParamsList)}
      , m_frame{&frame}
      , m_results{std::move(results)} {}

  FakeDepthQualityAssessor(const FakeDepthQualityAssessor &other) = delete;
  FakeDepthQualityAssessor(FakeDepthQualityAssessor &&other) = default;
  auto operator=(const FakeDepthQualityAssessor &other) -> FakeDepthQualityAssessor & = delete;
  auto operator=(FakeDepthQualityAssessor &&other) -> FakeDepthQualityAssessor & = default;
  ~FakeDepthQualityAssessor() final { REQUIRE(m_results.empty()); }

  auto isLowDepthQuality(const TMIV::MivBitstream::ViewParamsList &viewParamsList,
                         const TMIV::Common::DeepFrameList &frame) const -> bool final {
    CHECK(viewParamsList == m_viewParamsList);
    CHECK(&frame == m_frame);
    REQUIRE(!m_results.empty());

    const bool result = m_results.front();
    m_results.erase(m_results.begin());
    return result;
  }

private:
  TMIV::MivBitstream::ViewParamsList m_viewParamsList;
  const TMIV::Common::DeepFrameList *m_frame{nullptr};
  mutable std::vector<bool> m_results;
};
} // namespace test

namespace TMIV::Encoder {
auto assessDepthQuality(const Configuration &config,
                        const DepthQualityAssessor::IDepthQualityAssessor &depthQualityAssessor,
                        const MivBitstream::SequenceConfig &sequenceConfig,
                        const Common::DeepFrameList &firstFrame) -> bool;
} // namespace TMIV::Encoder

TEST_CASE("assessDepthQuality typically defers to the depth quality assessor") {
  const auto result = GENERATE(true, false);

  const auto firstFrame = TMIV::Common::DeepFrameList{};

  CHECK(TMIV::Encoder::assessDepthQuality(
            test::configuration1(),
            test::FakeDepthQualityAssessor{
                test::sequenceConfig1.sourceViewParams(), firstFrame, {result}},
            test::sequenceConfig1, firstFrame) == result);
}

TEST_CASE("assessDepthQuality does not call the assessor when the depth quality is specified in "
          "the configuration") {
  const auto result = GENERATE(true, false);
  const auto haveGeometry = GENERATE(true, false);

  const auto config = [=]() {
    auto x = test::configuration1();
    x.depthLowQualityFlag = result;
    x.haveGeometry = haveGeometry;
    return x;
  }();

  CHECK(TMIV::Encoder::assessDepthQuality(config, test::FakeDepthQualityAssessor{},
                                          test::sequenceConfig1, {}) == result);
}

TEST_CASE("assessDepthQuality returns false without calling the assessor when the encoder is "
          "configured not to have geometry video data") {
  const auto config = [=]() {
    auto x = test::configuration1();
    x.haveGeometry = false;
    return x;
  }();

  CHECK_FALSE(TMIV::Encoder::assessDepthQuality(config, test::FakeDepthQualityAssessor{},
                                                test::sequenceConfig1, {}));
}

namespace TMIV::Encoder {
auto createEncoderParams(const Configuration &config,
                         const MivBitstream::SequenceConfig &sequenceConfig,
                         const MivBitstream::ViewParamsList &viewParamsList,
                         bool depthLowQualityFlag) -> EncoderParams;
} // namespace TMIV::Encoder

TEST_CASE("createEncoderParams sets multiple syntax elements to hard-coded values") {
  const auto haveGeometry = GENERATE(false, true);
  const auto haveOccupancy = GENERATE(false, true);
  const auto haveTexture = GENERATE(false, true);

  if (!haveGeometry && !haveTexture) {
    return;
  }

  const auto geometryScaleEnabledFlag = GENERATE(false, true);

  if (!haveGeometry && geometryScaleEnabledFlag) {
    return;
  }

  const auto config = [=]() {
    auto x = test::configuration1();
    x.haveGeometry = haveGeometry;
    x.geometryScaleEnabledFlag = geometryScaleEnabledFlag;
    x.haveOccupancy = haveOccupancy;
    x.haveTexture = haveTexture;

    if (haveOccupancy) {
      x.occBitDepth = 10;
    }
    if (haveGeometry) {
      x.geoBitDepth = 10;
    }
    if (haveTexture) {
      x.texBitDepth = 10;
    }
    return x;
  }();

  const auto params = TMIV::Encoder::createEncoderParams(
      config, test::sequenceConfig1, test::sequenceConfig1.sourceViewParams(), true);

  CHECK(params.vps.profile_tier_level().ptl_level_idc() ==
        TMIV::MivBitstream::PtlLevelIdc::Level_3_5);
  CHECK(params.vps.profile_tier_level().ptl_profile_reconstruction_idc() ==
        TMIV::MivBitstream::PtlProfileReconstructionIdc::Rec_Unconstrained);

  for (uint8_t k = 0; k <= params.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = params.vps.vps_atlas_id(k);

    if (haveGeometry) {
      const auto &gi = params.vps.geometry_information(j);
      CHECK(gi.gi_geometry_codec_id() == 0);
      CHECK(gi.gi_geometry_2d_bit_depth_minus1() == 9);
      CHECK_FALSE(gi.gi_geometry_MSB_align_flag());
    }
    if (haveOccupancy) {
      const auto &oi = params.vps.occupancy_information(j);
      CHECK(oi.oi_occupancy_codec_id() == 0);
      CHECK(oi.oi_lossy_occupancy_compression_threshold() == 128);
      CHECK(oi.oi_occupancy_2d_bit_depth_minus1() == 9);
      CHECK_FALSE(oi.oi_occupancy_MSB_align_flag());
    }
    if (haveTexture) {
      const auto &ai = params.vps.attribute_information(j);
      CHECK(ai.ai_attribute_type_id(0) == TMIV::MivBitstream::AiAttributeTypeId::ATTR_TEXTURE);
      CHECK(ai.ai_attribute_dimension_minus1(0) == 2);
      CHECK(ai.ai_attribute_2d_bit_depth_minus1(0) == 9);
    }
  }

  const auto &vui = params.casps.casps_miv_extension().vui_parameters();
  CHECK_FALSE(vui.vui_poc_proportional_to_timing_flag());
  CHECK_FALSE(vui.vui_hrd_parameters_present_flag());
  CHECK(vui.coordinate_system_parameters() == TMIV::MivBitstream::CoordinateSystemParameters{});

  for (const auto &atlas : params.atlas) {
    CHECK(atlas.asps.asps_use_eight_orientations_flag());
    CHECK(atlas.asps.asps_extended_projection_enabled_flag());
    CHECK(atlas.asps.asps_normal_axis_limits_quantization_enabled_flag());
    CHECK(atlas.asps.asps_num_ref_atlas_frame_lists_in_asps() == 1);

    const auto &asme = atlas.asps.asps_miv_extension();

    if (geometryScaleEnabledFlag) {
      CHECK(asme.asme_geometry_scale_factor_x_minus1() == 1);
      CHECK(asme.asme_geometry_scale_factor_y_minus1() == 1);
    }

    CHECK(atlas.ath.ath_type() == TMIV::MivBitstream::AthType::I_TILE);
    CHECK(atlas.ath.ath_ref_atlas_frame_list_asps_flag());
  }
}

TEST_CASE("createEncoderParams can be configured to allocate an atlas for each view") {
  const auto config = [=]() {
    auto x = test::configuration1();
    x.oneViewPerAtlasFlag = true;
    return x;
  }();

  const auto sourceViewParams = test::sequenceConfig1.sourceViewParams();
  const auto params =
      TMIV::Encoder::createEncoderParams(config, test::sequenceConfig1, sourceViewParams, true);

  CHECK(params.vps.vps_atlas_count_minus1() + size_t{1} == sourceViewParams.size());

  for (uint8_t k = 0; k <= params.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = params.vps.vps_atlas_id(k);

    CHECK(params.vps.vps_frame_width(j) ==
          sourceViewParams[k].ci.ci_projection_plane_width_minus1() + 1);
    CHECK(params.vps.vps_frame_height(j) ==
          sourceViewParams[k].ci.ci_projection_plane_height_minus1() + 1);
  }
}

TEST_CASE("createEncoderParams can be configured to override atlas frame sizes") {
  const auto atlasFrameSizes =
      std::vector{TMIV::Common::Vec2i{100, 200}, TMIV::Common::Vec2i{300, 400},
                  TMIV::Common::Vec2i{500, 600}, TMIV::Common::Vec2i{700, 800}};

  const auto config = [=]() {
    auto x = test::configuration1();
    x.overrideAtlasFrameSizes = atlasFrameSizes;
    return x;
  }();

  const auto params = TMIV::Encoder::createEncoderParams(
      config, test::sequenceConfig1, test::sequenceConfig1.sourceViewParams(), true);

  CHECK(params.vps.vps_atlas_count_minus1() + size_t{1} == atlasFrameSizes.size());

  for (uint8_t k = 0; k <= params.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = params.vps.vps_atlas_id(k);

    CHECK(params.vps.vps_frame_width(j) == atlasFrameSizes[k].x());
    CHECK(params.vps.vps_frame_height(j) == atlasFrameSizes[k].y());
  }
}

TEST_CASE("createEncoderParams warns when the automatically derived atlas frame sizes have an "
          "aspect ratio outside of the HEVC range") {
  const auto viewParamsList = []() {
    auto x = test::sequenceConfig1.sourceViewParams();
    for (auto &vp : x) {
      vp.ci.ci_projection_plane_width_minus1(995); // manually tuned edge case
    }
    return x;
  }();

  // NOTE(BK): There is no dependency injection for the logging. As a fallback, inspect code
  // coverage and/or perform a visual inspection to check that the waring is issued.
  CHECK_NOTHROW(TMIV::Encoder::createEncoderParams(test::configuration1(), test::sequenceConfig1,
                                                   viewParamsList, true));
}

TEST_CASE("createEncoderParams assigns all atlases to the first group when grouping") {
  const auto numGroups = GENERATE(uint8_t{1}, uint8_t{2});

  const auto config = [=]() {
    auto x = test::configuration1();
    x.numGroups = numGroups;
    return x;
  }();

  const auto params = TMIV::Encoder::createEncoderParams(
      config, test::sequenceConfig1, test::sequenceConfig1.sourceViewParams(), true);

  const auto &gm = params.vps.vps_miv_extension().group_mapping();
  CHECK(gm.gm_group_count() == numGroups);

  for (uint8_t k = 0; k <= params.vps.vps_atlas_count_minus1(); ++k) {
    CHECK(gm.gm_group_id(k) == 0);
  }
}

TEST_CASE("createEncoderParams takes the attribute offset bit depth from the configuration") {
  const auto attributeOffsetBitCount = GENERATE(1, 5, 32);

  const auto config = [=]() {
    auto x = test::configuration1();
    x.attributeOffsetFlag = true;
    x.attributeOffsetBitCount = attributeOffsetBitCount;
    return x;
  }();

  const auto params = TMIV::Encoder::createEncoderParams(
      config, test::sequenceConfig1, test::sequenceConfig1.sourceViewParams(), true);

  for (const auto &atlas : params.atlas) {
    CHECK(atlas.asps.asps_miv_extension().asme_patch_attribute_offset_bit_depth_minus1() ==
          static_cast<uint16_t>(attributeOffsetBitCount - 1));
  }
}

TEST_CASE("createEncoderParams can embed the viewport camera parameters SEI message") {
  const auto viewportCameraParametersSei = GENERATE(false, true);

  const auto config = [=]() {
    auto x = test::configuration1();
    x.viewportCameraParametersSei = viewportCameraParametersSei;
    return x;
  }();

  const auto params = TMIV::Encoder::createEncoderParams(
      config, test::sequenceConfig1, test::sequenceConfig1.sourceViewParams(), true);

  CHECK(params.viewportCameraParameters.has_value() == viewportCameraParametersSei);
}

TEST_CASE("createEncoderParams can embed the viewport position SEI message") {
  const auto viewportPositionSei = GENERATE(false, true);

  const auto config = [=]() {
    auto x = test::configuration1();
    x.viewportPositionSei = viewportPositionSei;
    return x;
  }();

  const auto params = TMIV::Encoder::createEncoderParams(
      config, test::sequenceConfig1, test::sequenceConfig1.sourceViewParams(), true);

  CHECK(params.viewportPosition.has_value() == viewportPositionSei);
}

TEST_CASE("createEncoderParams sets patch size quantizers when necessary") {
  const auto blockSize = GENERATE(4, 8, 16, 32, 64, 128);
  const auto factor = GENERATE(1, 2, 4);
  const auto sizeQuantizer = blockSize / factor;
  CAPTURE(blockSize, sizeQuantizer);

  const auto config = [=]() {
    auto x = test::configuration1();
    x.blockSizeDepthQualityDependent = TMIV::Common::Vec2i{blockSize, blockSize};
    return x;
  }();

  const auto viewParamsList = [=]() {
    auto x = test::sequenceConfig1.sourceViewParams();
    x.front().ci.ci_projection_plane_width_minus1(1024 + sizeQuantizer - 1);
    return x;
  }();

  const auto params =
      TMIV::Encoder::createEncoderParams(config, test::sequenceConfig1, viewParamsList, true);

  for (const auto &atlas : params.atlas) {
    if (factor == 1) {
      REQUIRE_FALSE(atlas.asps.asps_patch_size_quantizer_present_flag());
    } else {
      REQUIRE(atlas.asps.asps_patch_size_quantizer_present_flag());
      CHECK(atlas.ath.ath_patch_size_x_info_quantizer() == TMIV::Common::ceilLog2(sizeQuantizer));
      CHECK(atlas.ath.ath_patch_size_y_info_quantizer() == TMIV::Common::ceilLog2(sizeQuantizer));
    }
  }
}
