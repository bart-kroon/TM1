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

#include <TMIV/BitstreamMerger/BitstreamMerger.h>
#include <TMIV/Common/Application.h>
#include <TMIV/Common/format.h>
#include <TMIV/IO/IO.h>
#include <TMIV/MivBitstream/Formatters.h>

#include <filesystem>
#include <fstream>
#include <numeric>
#include <stdexcept>

using namespace std::string_view_literals;

namespace TMIV::BitstreamMerger {
class BitstreamMergerApplication : public Common::Application {
private:
  const std::string &m_contentId;
  int32_t m_numberOfInputFrames;
  const std::string &m_testId;
  std::ifstream m_istream;
  std::ifstream m_imstream;
  std::unique_ptr<BitstreamMerger> m_bitstreamMerger{};

  [[nodiscard]] auto placeholders() const {
    auto x = IO::Placeholders{};
    x.contentId = m_contentId;
    x.numberOfInputFrames = m_numberOfInputFrames;
    x.testId = m_testId;

    return x;
  }

public:
  explicit BitstreamMergerApplication(std::vector<const char *> argv)
      : Common::Application{"BitstreamMerger",
                            std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false},
                                {"-r", "Test point (e.g. QP3 or R0)", false}},
                            {}}
      , m_contentId{optionValues("-s").front()}
      , m_numberOfInputFrames{std::stoi(optionValues("-n"sv).front())}
      , m_testId{optionValues("-r").front()}
      , m_bitstreamMerger{std::make_unique<BitstreamMerger>()} {}

  void run() override {
    json().checkForUnusedKeys();
    readInputBitstream();
    readOutOfBandMetadata();
    updatePackingInformation();
    writeOutputBitstream();
    writeOutOfBandMetadata();
  }

private:
  void readInputBitstream() {
    const auto path = IO::inputBitstreamPath(json(), placeholders());
    m_istream.open(path, std::ios::binary);
    if (m_istream.good()) {
      m_bitstreamMerger->readInputBitstream(m_istream);
    } else {
      throw std::runtime_error(
          TMIV_FMT::format("Failed to open input bitstream {} for reading.", path));
    }
  }

  void readOutOfBandMetadata() {
    const auto path = IO::inputBitstreamPath(json(), placeholders()).replace_extension(".json");
    m_imstream.open(path, std::ios::binary);
    if (m_imstream.good()) {
      const auto metadata = Common::Json::loadFrom(m_imstream);
      m_bitstreamMerger->readOutofBandMetadata(metadata.as<Common::Json::Array>().at(0));
    } else {
      throw std::runtime_error(
          TMIV_FMT::format("Failed to open the out-of-band metadata {}.", path));
    }
  }

  void updatePackingInformation() { m_bitstreamMerger->updatePackingInformation(); }

  void writeOutputBitstream() const {
    const auto path = IO::outputBitstreamPath(json(), placeholders());
    std::ofstream stream{path, std::ios::binary};
    if (!stream.good()) {
      throw std::runtime_error(TMIV_FMT::format("Failed to open {} for writing", path));
    }
    m_bitstreamMerger->writeOutputBitstream(stream);
    m_bitstreamMerger->reportSummary(stream.tellp());
  }

  void writeOutOfBandMetadata() const {
    const auto path = IO::outputBitstreamPath(json(), placeholders()).replace_extension(".json");
    std::ofstream stream{path, std::ios::binary};
    if (!stream.good()) {
      throw std::runtime_error(TMIV_FMT::format("Failed to open {} for writing", path));
    }
    m_bitstreamMerger->writeOutOfBandMetadata(stream);
  }
};
} // namespace TMIV::BitstreamMerger

auto main(int argc, char *argv[]) -> int32_t {
  try {
    TMIV::BitstreamMerger::BitstreamMergerApplication app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (...) {
    return TMIV::Common::handleException();
  }
}
