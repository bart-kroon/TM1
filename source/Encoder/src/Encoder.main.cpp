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

#include <TMIV/Encoder/IEncoder.h>

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Encoder/MivEncoder.h>
#include <TMIV/IO/IO.h>

#include <fstream>
#include <iostream>

using namespace std::string_view_literals;

namespace TMIV::Encoder {
void registerComponents();

class Application : public Common::Application {
private:
  std::unique_ptr<IEncoder> m_encoder;
  const std::string &m_contentId;
  std::int32_t m_numberOfInputFrames;
  std::int32_t m_intraPeriod;

  MivBitstream::SequenceConfig m_inputSequenceConfig;
  std::filesystem::path m_outputBitstreamPath;
  std::ofstream m_outputBitstream;
  std::unique_ptr<Encoder::MivEncoder> m_mivEncoder;

  [[nodiscard]] auto placeholders() const {
    auto x = IO::Placeholders{};
    x.contentId = m_contentId;
    x.numberOfInputFrames = m_numberOfInputFrames;
    return x;
  }

public:
  explicit Application(std::vector<const char *> argv)
      : Common::Application{"Encoder", std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false}}}
      , m_encoder{create<IEncoder>("Encoder")}
      , m_contentId{optionValues("-s"sv).front()}
      , m_numberOfInputFrames{std::stoi(optionValues("-n"sv).front())}
      , m_intraPeriod{json().require("intraPeriod").as<std::int32_t>()}
      , m_inputSequenceConfig{IO::loadSequenceConfig(json(), placeholders(), 0)}
      , m_outputBitstreamPath{IO::outputBitstreamPath(json(), placeholders())}
      , m_outputBitstream{m_outputBitstreamPath, std::ios::binary} {
    if (!m_outputBitstream.good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for writing", m_outputBitstreamPath));
    }

    // Support experiments that use a subset of the source cameras
    if (const auto &node = json().optional(IO::inputCameraNames)) {
      std::cout << "WARNING: Source camera names are derived from the sequence configuration. This "
                   "functionality to override source camera names is only for internal testing, "
                   "e.g. to test with a subset of views.\n";
      m_inputSequenceConfig.sourceCameraNames = node.asVector<std::string>();
    }

    m_mivEncoder = std::make_unique<MivEncoder>(m_outputBitstream);
  }

  void run() override {
    m_encoder->prepareSequence(
        m_inputSequenceConfig,
        IO::loadMultiviewFrame(json(), placeholders(), m_inputSequenceConfig, 0));

    for (int i = 0; i < m_numberOfInputFrames; i += m_intraPeriod) {
      int lastFrame = std::min(m_numberOfInputFrames, i + m_intraPeriod);
      encodeAccessUnit(i, lastFrame);
    }

    reportSummary(m_outputBitstream.tellp());
  }

private:
  void encodeAccessUnit(std::int32_t firstFrame, std::int32_t lastFrame) {
    std::cout << "Access unit: [" << firstFrame << ", " << lastFrame << ")\n";
    m_encoder->prepareAccessUnit();
    pushFrames(firstFrame, lastFrame);
    m_mivEncoder->writeAccessUnit(m_encoder->completeAccessUnit());
    popAtlases(firstFrame, lastFrame);
  }

  void pushFrames(std::int32_t firstFrame, std::int32_t lastFrame) {
    for (std::int32_t i = firstFrame; i < lastFrame; ++i) {
      m_encoder->pushFrame(
          IO::loadMultiviewFrame(json(), placeholders(), m_inputSequenceConfig, i));
    }
  }

  void popAtlases(int firstFrame, int lastFrame) {
    for (std::int32_t i = firstFrame; i < lastFrame; ++i) {
      IO::saveAtlasFrame(json(), placeholders(), i, m_encoder->popAtlas());
    }
  }

  void reportSummary(std::streampos bytesWritten) const {
    fmt::print("Maximum luma samples per frame is {}\n", m_encoder->maxLumaSamplesPerFrame());
    fmt::print("Total size is {} B ({} kb)\n", bytesWritten,
               8e-3 * static_cast<double>(bytesWritten));
    fmt::print("Frame count is {}\n", m_numberOfInputFrames);
    fmt::print("Frame rate is {} Hz\n", m_inputSequenceConfig.frameRate);
    fmt::print("Total bitrate is {} kbps\n", 8e-3 * static_cast<double>(bytesWritten) *
                                                 m_inputSequenceConfig.frameRate /
                                                 m_numberOfInputFrames);
  }
};
} // namespace TMIV::Encoder

auto main(int argc, char *argv[]) -> int {
  try {
    TMIV::Encoder::registerComponents();
    TMIV::Encoder::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
