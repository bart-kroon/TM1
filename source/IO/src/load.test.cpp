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

#include "test.h"

TEST_CASE("TMIV::IO::loadMultiviewFrame") {
  using TMIV::Common::ColorFormat;
  using TMIV::Common::Json;
  using TMIV::Common::Vec2i;
  using TMIV::MivBitstream::SequenceConfig;

  auto filesystem = test::injectFakeFilesystem();

  size_t truncate = GENERATE(0, 1);

  auto seqConfig = SequenceConfig{};
  seqConfig.cameras.emplace_back().viewParams.name = "name"s;
  seqConfig.cameras.back()
      .viewParams.ci.ci_projection_plane_width_minus1(3)
      .ci_projection_plane_height_minus1(5);
  seqConfig.sourceCameraNames.push_back("name"s);

  SECTION("Texture component") {
    seqConfig.cameras.back().bitDepthTexture = 7;
    seqConfig.cameras.back().colorFormatTexture = ColorFormat::YUV444;

    filesystem->fileData(test::dir1() / "tex_7_seq_rate_name_4x6_yuv444p7.yuv",
                         std::string(20 * 24 * 3 - truncate, 'z'));

    const auto config = Json::parse(R"({
	"inputDirectory": "fake",
	"inputTexturePathFmt": "tex_{0}_{1}_{2}_{3}_{4}x{5}_{6}.yuv"
})"sv);

    if (truncate == 1) {
      REQUIRE_THROWS(loadMultiviewFrame(config, test::placeholders(), seqConfig, 0));
    } else {
      auto result = loadMultiviewFrame(config, test::placeholders(), seqConfig, 0);

      REQUIRE(result.size() == 1);
      REQUIRE_FALSE(result.front().texture.empty());
      CHECK(result.front().texture.getSize() == Vec2i{4, 6});
      CHECK(result.front().texture.getColorFormat() == ColorFormat::YUV420); // Converted
      CHECK(result.front().texture.getBitDepth() == 7);
    }
  }

  SECTION("Geometry component") {
    seqConfig.cameras.back().bitDepthGeometry = 8;
    seqConfig.cameras.back().colorFormatGeometry = ColorFormat::YUV420;

    filesystem->fileData(test::dir1() / "geo_7_seq_rate_name_4x6_yuv420p.yuv",
                         std::string(20 * 24 * 3 / 2 - truncate, 'z'));

    const auto config = Json::parse(R"({
	"inputDirectory": "fake",
	"inputGeometryPathFmt": "geo_{0}_{1}_{2}_{3}_{4}x{5}_{6}.yuv"
})"sv);

    if (truncate == 1) {
      REQUIRE_THROWS(loadMultiviewFrame(config, test::placeholders(), seqConfig, 0));
    } else {
      auto result = loadMultiviewFrame(config, test::placeholders(), seqConfig, 0);

      REQUIRE(result.size() == 1);
      REQUIRE_FALSE(result.front().geometry.empty());
      CHECK(result.front().geometry.getSize() == Vec2i{4, 6});
      CHECK(result.front().geometry.getColorFormat() == ColorFormat::YUV400); // Converted
      CHECK(result.front().geometry.getBitDepth() == 16);                     // Requantized
    }
  }

  SECTION("Transparency component") {
    seqConfig.cameras.back().bitDepthTransparency = 4;
    seqConfig.cameras.back().colorFormatTransparency = ColorFormat::YUV444;

    filesystem->fileData(test::dir1() / "tra_7_seq_rate_name_4x6_yuv444p4.yuv",
                         std::string(20 * 24 * 3 - truncate, 'z'));

    const auto config = Json::parse(R"({
	"inputDirectory": "fake",
	"inputTransparencyPathFmt": "tra_{0}_{1}_{2}_{3}_{4}x{5}_{6}.yuv"
})"sv);

    if (truncate == 1) {
      REQUIRE_THROWS(loadMultiviewFrame(config, test::placeholders(), seqConfig, 0));
    } else {
      auto result = loadMultiviewFrame(config, test::placeholders(), seqConfig, 0);

      REQUIRE(result.size() == 1);
      REQUIRE_FALSE(result.front().transparency.empty());
      CHECK(result.front().transparency.getSize() == Vec2i{4, 6});
      CHECK(result.front().transparency.getColorFormat() == ColorFormat::YUV400); // Converted
      CHECK(result.front().transparency.getBitDepth() == 4);
    }
  }

  SECTION("Entity map") {
    seqConfig.cameras.back().bitDepthEntities = 4;
    seqConfig.cameras.back().colorFormatEntities = ColorFormat::YUV420;

    filesystem->fileData(test::dir1() / "ent_7_seq_rate_name_4x6_yuv420p4.yuv",
                         std::string(20 * 24 * 3 / 2 - truncate, 'z'));

    const auto config = Json::parse(R"({
	"inputDirectory": "fake",
	"inputEntityPathFmt": "ent_{0}_{1}_{2}_{3}_{4}x{5}_{6}.yuv"
})"sv);

    if (truncate == 1) {
      REQUIRE_THROWS(loadMultiviewFrame(config, test::placeholders(), seqConfig, 0));
    } else {
      auto result = loadMultiviewFrame(config, test::placeholders(), seqConfig, 0);

      REQUIRE(result.size() == 1);
      REQUIRE_FALSE(result.front().entities.empty());
      CHECK(result.front().entities.getSize() == Vec2i{4, 6});
      CHECK(result.front().entities.getColorFormat() == ColorFormat::YUV400); // Converted
      CHECK(result.front().entities.getBitDepth() == 4);
    }
  }
}

TEST_CASE("TMIV::IO::loadViewportMetadata") {
  using TMIV::Common::Json;
  using TMIV::MivBitstream::CiCamType;

  auto filesystem = test::injectFakeFilesystem();

  SECTION("Source view") {
    filesystem->fileData(test::dir1() / "viewportParams_7_seq_rate.json", R"({
    "BoundingBox_center": [ 0, 0, 0 ],
    "Content_name": "",
    "Fps": 0,
    "Frames_number": 0,
    "Version": "4.0",
    "cameras": [
        {
            "Name": "name",
            "Resolution": [ 1920, 1080 ],
            "Position": [ 1.0, 2.0, 3.0 ],
            "Rotation": [ 4.0, 5.0, 6.0 ],
            "Depth_range": [ 7.0, 8.0 ],
            "Projection": "Orthographic",
            "OrthoWidth": 9.0,
            "OrthoHeight": 10.0
        }
    ],
    "lengthsInMeters": true
})"s);

    const auto config = Json::parse(R"({
        "configDirectory": "fake",
        "inputViewportParamsPathFmt": "viewportParams_{0}_{1}_{2}.json"
})"sv);

    const auto result = loadViewportMetadata(config, test::placeholders(), 0, "name", false);

    CHECK(result.viewParams.ci.ci_cam_type() == CiCamType::orthographic);
  }

  SECTION("Pose trace") {
    using TMIV::Common::QuatD;
    using TMIV::Common::Vec3f;

    filesystem->fileData(test::dir1() / "viewportParams_7_seq_rate.json", R"({
    "BoundingBox_center": [ 0, 0, 0 ],
    "Content_name": "",
    "Fps": 0,
    "Frames_number": 0,
    "Version": "4.0",
    "cameras": [
        {
            "Name": "viewport",
            "Resolution": [ 1920, 1080 ],
            "Position": [ 1.0, 2.0, 3.0 ],
            "Rotation": [ 10.0, 20.0, 30.0 ],
            "Depth_range": [ 7.0, 8.0 ],
            "Projection": "Orthographic",
            "OrthoWidth": 9.0,
            "OrthoHeight": 10.0
        }
    ],
    "lengthsInMeters": true
})"s);

    filesystem->fileData(test::dir1() / "posetrace_7_seq_rate_name.csv", R"(X,Y,Z,Yaw,Pitch,Roll
0.1,0.2,0.3,45,-45,90
1.1,1.2,1.3,1.4,1.5,1.6


)");

    const auto config = Json::parse(R"({
        "configDirectory": "fake",
        "inputViewportParamsPathFmt": "viewportParams_{0}_{1}_{2}.json",
        "inputPoseTracePathFmt": "posetrace_{0}_{1}_{2}_{3}.csv"
})"sv);

    const auto result = loadViewportMetadata(config, test::placeholders(), 0, "name", true);

    CHECK(result.viewParams.pose.position == Vec3f{1.1F, 2.2F, 3.3F});

    // The orientation of the viewport is ignored
    CHECK(result.viewParams.pose.orientation.x() == Catch::Approx(std::sqrt(0.5)));
    CHECK(result.viewParams.pose.orientation.y() == 0.0);
    CHECK(result.viewParams.pose.orientation.z() == 0.5);
    CHECK(result.viewParams.pose.orientation.w() == Catch::Approx(0.5));
  }
}

TEST_CASE("TMIV::IO::loadOutOfBandVideoFrame") {
  using TMIV::Common::ColorFormat;
  using TMIV::Common::Json;
  using TMIV::Common::Vec2i;
  using TMIV::MivBitstream::AiAttributeTypeId;
  using TMIV::MivBitstream::AtlasId;
  using TMIV::MivBitstream::AtlasSequenceParameterSetRBSP;
  using TMIV::MivBitstream::V3cParameterSet;
  using TMIV::MivBitstream::V3cUnitHeader;

  auto filesystem = test::injectFakeFilesystem();

  size_t truncate = GENERATE(0, 1);

  filesystem->fileData(test::dir1() / "nor_7_seq_rate_3_4x6_yuv420p6.yuv",
                       std::string(24 * 3 / 2 - truncate, '_'));

  filesystem->fileData(test::dir1() / "TMIV_7_seq_rate.json", R"(
    [
        {
            "vuh_unit_type": 4,
            "vuh_v3c_parameter_set_id": 2,
            "vuh_atlas_id": 3,
            "vuh_attribute_index": 4,
            "vuh_attribute_partition_index": 0,
            "vuh_map_index": 0,
            "vuh_auxiliary_video_flag": false,
            "ai_attribute_type_id": 4,
            "frame_size": [ 4, 6 ],
            "bit_depth": 6,
            "irap_frame_indices": [ 0, 5, 13 ]
        }
    ]
)"s);

  const auto config = Json::parse(R"({
      "inputDirectory": "fake",
      "inputBitstreamPathFmt": "TMIV_{0}_{1}_{2}.bit",
      "inputNormalVideoFramePathFmt": "nor_{0}_{1}_{2}_{3}_{4}x{5}_{6}.yuv"
})"sv);

  V3cParameterSet vps;

  const auto atlasId = AtlasId{3};
  const auto attrIdx = uint8_t{4};
  const auto vuh = V3cUnitHeader::avd(2, atlasId, attrIdx);

  if (truncate == 1) {
    REQUIRE_THROWS(loadOutOfBandVideoFrame(config, test::placeholders(), vuh, 0));
  } else {
    const auto result = loadOutOfBandVideoFrame(config, test::placeholders(), vuh, 0);

    REQUIRE_FALSE(result.empty());
    CHECK(result.getSize() == Vec2i{4, 6});
    CHECK(result.getBitDepth() == 6);
    CHECK(result.getColorFormat() == ColorFormat::YUV420);
    CHECK(result.irap);
  }
}

TEST_CASE("TMIV::IO::loadSequenceConfig") {
  using TMIV::Common::Json;

  const auto config = Json::parse(R"({
      "inputDirectory": "fake",
      "configDirectory": "mirage",
      "inputSequenceConfigPathFmt": "sub/seqConfig_{0}_{1}_{2}_{3}.json"
})"sv);

  auto filesystem = test::injectFakeFilesystem();

  const auto haveFile = GENERATE(false, true);

  if (haveFile) {
    // The function at test probes the input and configuration directory
    const auto path = GENERATE(test::dir1() / "sub/seqConfig_7_seq_rate_43.json",
                               test::dir2() / "sub/seqConfig_7_seq_rate_43.json");

    filesystem->fileData(path, R"({
    "BoundingBox_center": [ 0, 0, 0 ],
    "Content_name": "testing",
    "Fps": 0,
    "Frames_number": 0,
    "Version": "4.0",
    "cameras": [ ],
    "lengthsInMeters": true
})");
  }

  SECTION("loadSequenceConfig") {
    if (haveFile) {
      const auto result = loadSequenceConfig(config, test::placeholders(), 43);

      CHECK(result.contentName == "testing");
    } else {
      REQUIRE_THROWS(loadSequenceConfig(config, test::placeholders(), 43));
    }
  }

  SECTION("tryLoadSequenceConfig") {
    const auto result = tryLoadSequenceConfig(config, test::placeholders(), 43);

    REQUIRE(haveFile == result.has_value());

    if (haveFile) {
      CHECK(result->contentName == "testing");
    }
  }
}

TEST_CASE("TMIV::IO::loadMpiTextureMpiLayer") {
  using TMIV::Common::ColorFormat;
  using TMIV::Common::Json;
  using TMIV::Common::Vec2i;
  using TMIV::MivBitstream::SequenceConfig;

  auto filesystem = test::injectFakeFilesystem();

  size_t truncate = GENERATE(0, 1);

  filesystem->fileData(test::dir1() / "tex_7_seq_rate_name_4x6_gray.yuv",
                       std::string((11 + 19) * 24 - truncate, 'z'));

  const auto config = Json::parse(R"({
      "inputDirectory": "fake",
      "inputTexturePathFmt": "tex_{0}_{1}_{2}_{3}_{4}x{5}_{6}.yuv"
})"sv);

  auto seqConfig = SequenceConfig{};
  seqConfig.cameras.emplace_back().viewParams.name = "name"s;
  seqConfig.cameras.back()
      .viewParams.ci.ci_projection_plane_width_minus1(3)
      .ci_projection_plane_height_minus1(5);
  seqConfig.cameras.back().bitDepthTexture = 8;
  seqConfig.cameras.back().colorFormatTexture = ColorFormat::YUV400;
  seqConfig.sourceCameraNames.push_back("name"s);

  if (truncate == 1) {
    REQUIRE_THROWS(loadMpiTextureMpiLayer(config, test::placeholders(), seqConfig, 1, 2, 8));
  } else {
    const auto result = loadMpiTextureMpiLayer(config, test::placeholders(), seqConfig, 1, 2, 8);

    REQUIRE_FALSE(result.empty());
    CHECK(result.getSize() == Vec2i{4, 6});
    CHECK(result.getColorFormat() == ColorFormat::YUV400); // No conversion
    CHECK(result.getBitDepth() == 8);
  }
}

TEST_CASE("TMIV::IO::loadMpiTransparencyMpiLayer") {
  using TMIV::Common::ColorFormat;
  using TMIV::Common::Json;
  using TMIV::Common::Vec2i;
  using TMIV::MivBitstream::SequenceConfig;

  auto filesystem = test::injectFakeFilesystem();

  size_t truncate = GENERATE(0, 1);

  filesystem->fileData(test::dir1() / "tex_7_seq_rate_name_4x6_yuv420p4.yuv",
                       std::string((11 + 19) * 24 * 3 / 2 - truncate, 'z'));

  const auto config = Json::parse(R"({
      "inputDirectory": "fake",
      "inputTransparencyPathFmt": "tex_{0}_{1}_{2}_{3}_{4}x{5}_{6}.yuv"
})"sv);

  auto seqConfig = SequenceConfig{};
  seqConfig.cameras.emplace_back().viewParams.name = "name"s;
  seqConfig.cameras.back()
      .viewParams.ci.ci_projection_plane_width_minus1(3)
      .ci_projection_plane_height_minus1(5);
  seqConfig.cameras.back().bitDepthTransparency = 4;
  seqConfig.cameras.back().colorFormatTransparency = ColorFormat::YUV420;
  seqConfig.sourceCameraNames.push_back("name"s);

  if (truncate == 1) {
    REQUIRE_THROWS(loadMpiTransparencyMpiLayer(config, test::placeholders(), seqConfig, 1, 2, 8));
  } else {
    const auto result =
        loadMpiTransparencyMpiLayer(config, test::placeholders(), seqConfig, 1, 2, 8);

    REQUIRE_FALSE(result.empty());
    CHECK(result.getSize() == Vec2i{4, 6});
    CHECK(result.getColorFormat() == ColorFormat::YUV420); // No conversion
    CHECK(result.getBitDepth() == 4);
  }
}

TEST_CASE("TMIV::IO::inputBitstreamPath") {
  using TMIV::Common::Json;

  const auto config = Json::parse(R"({
      "inputDirectory": "fake",
      "inputBitstreamPathFmt": "bitstream_{0}_{1}_{2}.miv"
})"sv);

  const auto result = inputBitstreamPath(config, test::placeholders());

  CHECK(result == test::dir1() / "bitstream_7_seq_rate.miv");
}

TEST_CASE("TMIV::IO::inputVideoSubBitstreamPath") {
  using TMIV::Common::Json;
  using TMIV::MivBitstream::AiAttributeTypeId;
  using TMIV::MivBitstream::AtlasId;
  using TMIV::MivBitstream::V3cUnitHeader;

  SECTION("Packed video data") {
    const auto config = Json::parse(R"({
        "inputDirectory": "fake",
        "inputPackedVideoSubBitstreamPathFmt": "bitstream_{0}_{1}_{2}_{4}.hevc"
})"sv);

    const auto result =
        inputVideoSubBitstreamPath(config, test::placeholders(), V3cUnitHeader::pvd(4, AtlasId{3}));

    // The final placeholder (attribute index) is zero because there is none
    CHECK(result == test::dir1() / "bitstream_7_seq_rate_0.hevc");
  }

  SECTION("Attribute video data") {
    const auto config = Json::parse(R"({
        "inputDirectory": "fake",
        "inputMaterialIdVideoSubBitstreamPathFmt": "bitstream_{0}_{1}_{2}_{4}.hevc"
})"sv);

    const auto result = inputVideoSubBitstreamPath(config, test::placeholders(),
                                                   V3cUnitHeader::avd(4, AtlasId{3}, 22),
                                                   AiAttributeTypeId::ATTR_MATERIAL_ID);

    CHECK(result == test::dir1() / "bitstream_7_seq_rate_22.hevc");
  }
}
