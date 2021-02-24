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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

using Catch::Matchers::Contains;

#include <TMIV/Common/Frame.h>
#include <TMIV/MpiPcs/MpiPcs.h>
#include <sstream>

namespace TMIV::MpiPcs {
TEST_CASE("MpiPcs writer and reader") {

  SECTION("FileHeader") {
    std::vector<char> header_items = {'M', 'P', 'I', '_', 'P', 'C', 'S', '_', '1', '\0'};
    std::string expected_header(header_items.begin(), header_items.end());

    std::stringstream ss;
    MpiPcs::FileHeader::write(ss);
    REQUIRE(ss.str() == expected_header);

    REQUIRE_NOTHROW(MpiPcs::FileHeader::read(ss));

    std::stringstream ss2;
    ss2.str("wrong_header");
    REQUIRE_THROWS(MpiPcs::FileHeader::read(ss2));
  }

  SECTION("Writer 1") {
    Common::Vec2i sz{2, 2};
    Common::MpiPcs::Frame mpiPcsFrame(sz);

    auto mpiLayer = Common::TextureTransparency8Frame{Common::TextureFrame{2, 2},
                                                      Common::Transparency8Frame{2, 2}};

    mpiLayer.transparency.getPlane(0)[0] = 255;
    mpiPcsFrame.appendLayer(18, mpiLayer);
    std::stringstream ss;
    MpiPcs::Writer writer{};

    REQUIRE(writer.getPath() == "");

    // writing a frame
    writer.append(ss, mpiPcsFrame);
    std::streamoff expected_number_of_bytes = 8       // n1, n2, n3, n4 : one value for each pixel
                                              + 1 * ( // number of active pixels
                                                        6     // rgb
                                                        + 2   // layer id
                                                        + 1); // transparency

    ss.seekg(0, std::ios::end);
    REQUIRE(ss.tellg() == expected_number_of_bytes);
    ss.seekg(0, std::ios::beg);

    std::string expected_string = "0100"; // n1
    expected_string += "0000";            // n2
    expected_string += "0000";            // n3
    expected_string += "0000";            // n4
    expected_string += "000000000000";    // rgb
    expected_string += "1200";            // layer
    expected_string += "FF";              // transparency

    std::ostringstream ss2;
    for (std::streamoff idx = 0; idx < expected_number_of_bytes; idx++) {
      std::array<char, 1> n{};
      ss.read(n.data(), n.size());
      ss2 << fmt::format("{:02X}", std::uint8_t(n[0]));
    }
    REQUIRE(ss2.str() == expected_string);

    // adding a second frame
    expected_number_of_bytes = 34;
    expected_string += expected_string;
    writer.append(ss, mpiPcsFrame);
    ss.seekg(0, std::ios::end);
    REQUIRE(ss.tellg() == expected_number_of_bytes);
    ss.seekg(0, std::ios::beg);

    std::ostringstream ss3;
    for (std::streamoff idx = 0; idx < expected_number_of_bytes; idx++) {
      std::array<char, 1> n{};
      ss.read(n.data(), n.size());
      ss3 << fmt::format("{:02X}", std::uint8_t(n[0]));
    }
    REQUIRE(ss3.str() == expected_string);
  }

  SECTION("Writer 2") {
    // 3 different layers, 5 active pixels in total
    Common::Vec2i size{4, 2};
    Common::MpiPcs::Frame mpiPcsFrame(size);

    auto mpiLayer1 = Common::TextureTransparency8Frame{Common::TextureFrame{4, 2},
                                                       Common::Transparency8Frame{4, 2}};
    mpiLayer1.transparency.getPlane(0)[1] = 11;
    mpiLayer1.transparency.getPlane(0)[5] = 15;
    mpiPcsFrame.appendLayer(1, mpiLayer1);

    auto mpiLayer2 = Common::TextureTransparency8Frame{Common::TextureFrame{4, 2},
                                                       Common::Transparency8Frame{4, 2}};
    mpiLayer2.transparency.getPlane(0)[1] = 21;
    mpiLayer2.transparency.getPlane(0)[2] = 22;
    mpiPcsFrame.appendLayer(2, mpiLayer2);

    auto mpiLayer3 = Common::TextureTransparency8Frame{Common::TextureFrame{4, 2},
                                                       Common::Transparency8Frame{4, 2}};
    mpiLayer3.transparency.getPlane(0)[7] = 37;
    mpiPcsFrame.appendLayer(57, mpiLayer3);

    std::stringstream ss;
    MpiPcs::Writer writer{};

    writer.append(ss, mpiPcsFrame);

    std::streamoff expected_number_of_bytes = 16      // n1, n2, ..., n8 : one value for each pixel
                                              + 5 * ( // number of active pixels
                                                        6     // rgb
                                                        + 2   // layer id
                                                        + 1); // transparency

    ss.seekg(0, std::ios::end);
    REQUIRE(ss.tellg() == expected_number_of_bytes);
    ss.seekg(0, std::ios::beg);

    std::string expected_string = "00000200010000000000010000000100"; // n1, n2, ...n8
    expected_string += "00000000000001000B";                          // active pixel #1
    expected_string += "000000000000020015";                          // active pixel #2
    expected_string += "000000000000020016";                          // active pixel #3
    expected_string += "00000000000001000F";                          // active pixel #4
    expected_string += "000000000000390025";                          // active pixel #5

    std::ostringstream ss2;
    for (std::streamoff idx = 0; idx < expected_number_of_bytes; idx++) {
      std::array<char, 1> n{};
      ss.read(n.data(), n.size());
      ss2 << fmt::format("{:02X}", int(n[0]));
    }
    REQUIRE(ss2.str() == expected_string);
  }

  SECTION("Reader") {
    Common::Vec2i size{2, 2};
    Common::MpiPcs::Frame mpiPcsFrame(size);

    auto mpiLayer = Common::TextureTransparency8Frame{Common::TextureFrame{2, 2},
                                                      Common::Transparency8Frame{2, 2}};

    mpiLayer.transparency.getPlane(0)[0] = 255;
    mpiPcsFrame.appendLayer(18, mpiLayer);
    std::stringstream ss;
    MpiPcs::Writer writer{};

    // writing a frame
    writer.append(ss, mpiPcsFrame);

    // reading a frame
    auto config = Common::Json{std::in_place_type_t<Common::Json::Object>{},
                               std::pair{"inputDirectory"s, Common::Json{"C:/fakeDir/"}},
                               std::pair{"inputMpiPcsPathFmt"s, Common::Json{"fake.pcs"}}};

    auto ci = MivBitstream::CameraIntrinsics{};
    ci.ci_projection_plane_width_minus1(size.x() - 1);
    ci.ci_projection_plane_height_minus1(size.y() - 1);

    auto viewParams = MivBitstream::ViewParams{};
    viewParams.ci = ci;
    viewParams.name = "fake_name";

    auto cameraConfig = MivBitstream::CameraConfig{};
    cameraConfig.viewParams = viewParams;
    cameraConfig.bitDepthTransparency = 8;

    auto sequenceConfig = MivBitstream::SequenceConfig{};
    sequenceConfig.cameras.push_back(cameraConfig);
    sequenceConfig.sourceCameraNames.push_back(viewParams.name);

    MpiPcs::Reader reader{config, IO::Placeholders{}, sequenceConfig, false};

    REQUIRE(reader.getPath() == "C:/fakeDir/fake.pcs");

    Common::MpiPcs::Frame rd_mpiPcsFrame{reader.read(ss, 0, Common::Vec2i({2, 2}))};

    REQUIRE(rd_mpiPcsFrame.getPixelList().size() == mpiPcsFrame.getPixelList().size());
    REQUIRE(rd_mpiPcsFrame.getPixelList() == mpiPcsFrame.getPixelList());
  }
}

} // namespace TMIV::MpiPcs
