/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

TEST_CASE("TMIV::IO::saveOutOfBandVideoFrame") {
  using TMIV::Common::ColorFormat;
  using TMIV::Common::Json;
  using TMIV::MivBitstream::V3cUnitHeader;
  using ATI = TMIV::MivBitstream::AiAttributeTypeId;

  auto filesystem = test::injectFakeFilesystem();

  SECTION("Geometry video data") {
    const auto config = Json::parse(R"({
    "outputDirectory": "fake",
    "outputGeometryVideoDataPathFmt": "geo_{0}_{1}_{2}_{3}_{4}x{5}_{6}.yuv"
})"sv);

    saveOutOfBandVideoFrame(config, test::placeholders(),
                            test::frame({3, 4}, ColorFormat::YUV400, 7), V3cUnitHeader::gvd(0, {}),
                            0);

    const auto path = test::dir1() / "geo_7_seq_rate_0_3x4_gray7.yuv";
    REQUIRE(filesystem->haveFile(path));
    CHECK(filesystem->fileData(path) == "The quick br"s);
  }

  SECTION("Attribute video data") {
    const auto config = Json::parse(R"({
    "outputDirectory": "fake",
    "outputTextureVideoDataPathFmt":"tex_{0}_{1}_{2}_{3}_{4}x{5}_{6}.yuv"
})"sv);

    saveOutOfBandVideoFrame(config, test::placeholders(),
                            test::frame({4, 4}, ColorFormat::YUV420, 8),
                            V3cUnitHeader::avd(0, {}, 0), 0, ATI::ATTR_TEXTURE);

    const auto path = test::dir1() / "tex_7_seq_rate_0_4x4_yuv420p.yuv";
    REQUIRE(filesystem->haveFile(path));
    CHECK(filesystem->fileData(path) == "The quick brown The The "s);
  }
}

TEST_CASE("TMIV::IO::saveViewport") {
  using TMIV::Common::ColorFormat;
  using TMIV::Common::DeepFrame;
  using TMIV::Common::Json;

  auto filesystem = test::injectFakeFilesystem();

  auto deepFrame = DeepFrame{};
  deepFrame.texture = test::frame({4, 4}, ColorFormat::YUV444, 7);
  deepFrame.geometry = test::frame({4, 2}, ColorFormat::YUV400, 8);

  SECTION("Saving the viewport can be skipped (with a runtime warning)") {
    const auto config = Json::parse(R"({
        "outputDirectory": "fake"
})"sv);

    saveViewport(config, test::placeholders(), 0, "name", deepFrame);

    REQUIRE(filesystem->empty());
  }

  SECTION("Save only the texture component of the viewport") {
    const auto config = Json::parse(R"({
        "outputDirectory": "fake",
        "outputViewportTexturePathFmt": "tex_{0}_{1}_{2}_{3}_{4}_{5}x{6}_{7}.yuv"
})"sv);

    saveViewport(config, test::placeholders(), 0, "name", deepFrame);

    CHECK(filesystem->fileData(test::dir1() / "tex_7_seq_rate_11_name_4x4_yuv444p7.yuv") ==
          "The quick brown The quick brown The quick brown "s);
  }

  SECTION("Save only the geometry component of the viewport") {
    const auto config = Json::parse(R"({
        "outputDirectory": "fake",
        "outputViewportGeometryPathFmt": "geo_{0}_{1}_{2}_{3}_{4}_{5}x{6}_{7}.yuv"
})"sv);

    saveViewport(config, test::placeholders(), 0, "name", deepFrame);

    CHECK(filesystem->fileData(test::dir1() / "geo_7_seq_rate_11_name_4x2_gray.yuv") ==
          "The quic"s);
  }
}

TEST_CASE("TMIV::IO::optionalSaveBlockToPatchMaps") {
  using TMIV::Common::Json;
  using TMIV::MivBitstream::AccessUnit;

  auto filesystem = test::injectFakeFilesystem();

  auto frame = AccessUnit{};
  frame.atlas.emplace_back().blockToPatchMap.createY({4, 4});
  std::fill(frame.atlas.back().blockToPatchMap.getPlane(0).begin(),
            frame.atlas.back().blockToPatchMap.getPlane(0).end(), uint16_t{'/' << 8 | '-'});

  SECTION("Skip saving the block to patch map") {
    const auto config = Json::parse(R"({
        "outputDirectory": "fake"
})"sv);

    optionalSaveBlockToPatchMaps(config, test::placeholders(), 0, frame);

    REQUIRE(filesystem->empty());
  }

  SECTION("Save the block to patch map") {
    const auto config = Json::parse(R"({
        "outputDirectory": "fake",
        "outputBlockToPatchMapPathFmt": "btpm_{0}_{1}_{2}_{3}_{4}x{5}_{6}.yuv"
})"sv);

    optionalSaveBlockToPatchMaps(config, test::placeholders(), 0, frame);

    CHECK(filesystem->fileData(test::dir1() / "btpm_7_seq_rate_0_4x4_gray16le.yuv") ==
          "-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/"s);
  }
}

TEST_CASE("TMIV::IO::optionalSaveSequenceConfig") {
  using TMIV::Common::Json;
  using TMIV::MivBitstream::SequenceConfig;

  auto filesystem = test::injectFakeFilesystem();

  auto seqConfig = TMIV::MivBitstream::SequenceConfig{};

  SECTION("Skip saving the sequence configuration") {
    const auto config = Json::parse(R"({
        "outputDirectory": "fake"
})"sv);

    optionalSaveSequenceConfig(config, test::placeholders(), 0, seqConfig);

    REQUIRE(filesystem->empty());
  }

  SECTION("Save the sequence configuration") {
    const auto config = Json::parse(R"({
        "outputDirectory": "fake",
        "outputSequenceConfigPathFmt": "seqConfig_{0}_{1}_{2}_{3:04}.json"
})"sv);

    optionalSaveSequenceConfig(config, test::placeholders(), 0, seqConfig);

    CHECK(filesystem->fileData(test::dir1() / "seqConfig_7_seq_rate_0000.json") == R"({
    "BoundingBox_center": [ 0, 0, 0 ],
    "Content_name": "",
    "Fps": 0,
    "Frames_number": 0,
    "Version": "4.0",
    "cameras": [ ],
    "lengthsInMeters": true
})"s);
  }
}

TEST_CASE("TMIV::IO::outputBitstreamPath") {
  using TMIV::Common::Json;

  auto filesystem = test::injectFakeFilesystem();

  const auto config = Json::parse(R"({
      "outputDirectory": "fake",
      "outputBitstreamPathFmt": "subdir/bitstream_{0}_{1}_{2}.yuv"
})"sv);

  const auto result = outputBitstreamPath(config, test::placeholders());

  REQUIRE(result == test::dir1() / "subdir" / "bitstream_7_seq_rate.yuv");
  REQUIRE(filesystem->haveDir(test::dir1() / "subdir"));
}
