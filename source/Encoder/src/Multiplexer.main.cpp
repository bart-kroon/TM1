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

#include <TMIV/Common/Application.h>
#include <TMIV/Encoder/Multiplexer.h>
#include <TMIV/IO/IO.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>

using namespace std::string_view_literals;

namespace TMIV::Encoder {
class MultiplexerApplication : public Common::Application {
private:
  const std::string &m_contentId;
  int32_t m_numberOfInputFrames;
  const std::string &m_testId;

  std::unique_ptr<Multiplexer> m_multiplexer{};

  [[nodiscard]] auto placeholders() const {
    auto x = IO::Placeholders{};
    x.contentId = m_contentId;
    x.numberOfInputFrames = m_numberOfInputFrames;
    x.testId = m_testId;
    return x;
  }

public:
  explicit MultiplexerApplication(std::vector<const char *> argv)
      : Common::Application{"Multiplexer", std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false},
                                {"-r", "Test point (e.g. QP3 or R0)", false}}}
      , m_contentId{optionValues("-s").front()}
      , m_numberOfInputFrames{std::stoi(optionValues("-n"sv).front())}
      , m_testId{optionValues("-r").front()}
      , m_multiplexer{std::make_unique<Multiplexer>()} {
    setBitstreamIstreamServers();
  }

  void run() override {
    readInputBitstream();
    addPackingInformation();
    appendVideoSubBitstreams();
    writeOutputBitstream();
  }

  [[nodiscard]] auto openInputBitstream(const std::string &key,
                                        const MivBitstream::AtlasId &atlasId,
                                        int attributeIdx = 0) const {
    const auto path = IO::inputSubBitstreamPath(key, json(), placeholders(), atlasId, attributeIdx);
    auto stream = std::make_unique<std::ifstream>(path, std::ios::binary);
    if (stream->fail()) {
      throw std::runtime_error(fmt::format("Failed to open sub bitstream {} for reading", path));
    }
    std::cout << "Appending " << path << '\n';
    return stream;
  }

  [[nodiscard]] auto openGeometryBitstream(const MivBitstream::AtlasId &atlasId) const
      -> std::unique_ptr<std::istream> {
    return openInputBitstream(IO::inputGeometryVsbPathFmt, atlasId);
  }

  [[nodiscard]] auto openOccupancyBitstream(const MivBitstream::AtlasId &atlasId) const
      -> std::unique_ptr<std::istream> {
    return openInputBitstream(IO::inputOccupancyVsbPathFmt, atlasId);
  }

  [[nodiscard]] auto openPackedBitstream(const MivBitstream::AtlasId &atlasId) const
      -> std::unique_ptr<std::istream> {
    return openInputBitstream(IO::inputPackedVsbPathFmt, atlasId);
  }

  [[nodiscard]] auto openAttributeBitstream(MivBitstream::AiAttributeTypeId typeId,
                                            const MivBitstream::AtlasId &atlasId,
                                            int attributeIdx) const
      -> std::unique_ptr<std::istream> {
    const auto key = [typeId]() {
      switch (typeId) {
      case MivBitstream::AiAttributeTypeId::ATTR_TEXTURE:
        return IO::inputTextureVsbPathFmt;
      case MivBitstream::AiAttributeTypeId::ATTR_MATERIAL_ID:
        return IO::inputMaterialIdVsbPathFmt;
      case MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY:
        return IO::inputTransparencyVsbPathFmt;
      case MivBitstream::AiAttributeTypeId::ATTR_REFLECTANCE:
        return IO::inputReflectanceVsbPathFmt;
      case MivBitstream::AiAttributeTypeId::ATTR_NORMAL:
        return IO::inputNormalVsbPathFmt;
      default:
        throw std::runtime_error(fmt::format("No support for {}", typeId));
      }
    }();

    return openInputBitstream(key, atlasId, attributeIdx);
  }

  void setBitstreamIstreamServers() {
    m_multiplexer->setAttributeVideoBitstreamServer(
        [this](MivBitstream::AiAttributeTypeId typeId, const MivBitstream::AtlasId &atlasId,
               int attributeIdx) { return openAttributeBitstream(typeId, atlasId, attributeIdx); });

    m_multiplexer->setGeometryVideoBitstreamServer(
        [this](const MivBitstream::AtlasId &atlasId) { return openGeometryBitstream(atlasId); });

    m_multiplexer->setOccupancyVideoBitstreamServer(
        [this](const MivBitstream::AtlasId &atlasId) { return openOccupancyBitstream(atlasId); });

    m_multiplexer->setPackedVideoBitstreamServer(
        [this](const MivBitstream::AtlasId &atlasId) { return openPackedBitstream(atlasId); });
  }

private:
  void readInputBitstream() {
    const auto path = IO::inputBitstreamPath(json(), placeholders());
    std::ifstream stream{path, std::ios::binary};
    if (!stream.good()) {
      throw std::runtime_error(fmt::format("Failed to open input bitstream {} for reading.", path));
    }
    m_multiplexer->readInputBitstream(stream);
    std::cout << "Appended " << path << " with a total of " << m_multiplexer->numberOfV3cUnits()
              << " V3C units including the VPS\n";
  }

  void addPackingInformation() {
    bool packingInformationPresent = false;

    if (json().optional("packingInformationPresent")) {
      packingInformationPresent = json().optional("packingInformationPresent").as<bool>();
    }

    if (packingInformationPresent) {
      m_multiplexer->addPackingInformation();
    }
  }

  void appendVideoSubBitstreams() { m_multiplexer->appendVideoSubBitstreams(); }

  void writeOutputBitstream() const {
    const auto path = IO::outputBitstreamPath(json(), placeholders());
    std::ofstream stream{path, std::ios::binary};
    if (!stream.good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for writing", path));
    }
    m_multiplexer->writeOutputBitstream(stream);
  }
};
} // namespace TMIV::Encoder

auto main(int argc, char *argv[]) -> int {
  try {
    TMIV::Encoder::MultiplexerApplication app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
