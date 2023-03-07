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

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Encoder/EncodeMiv.h>
#include <TMIV/Encoder/MpiEncoder.h>
#include <TMIV/Encoder/V3cSampleSink.h>
#include <TMIV/MivBitstream/Formatters.h>
#include <TMIV/MpiPcs/MpiPcs.h>

#include <fstream>

using namespace std::string_view_literals;

namespace TMIV::Encoder {
void registerComponents();

class Application : public Common::Application {
private:
  MpiEncoder m_encoder;
  const std::string &m_contentId;
  int32_t m_numberOfInputFrames;
  int32_t m_startFrame;
  int32_t m_intraPeriod;

  MivBitstream::SequenceConfig m_inputSequenceConfig;
  MpiPcs::Reader m_mpiPcsReader;
  std::filesystem::path m_outputBitstreamPath;
  std::ofstream m_outputBitstream;
  Common::Sink<EncoderParams> m_sink;
  static constexpr uint8_t m_vpsId = 0;

  [[nodiscard]] auto placeholders() const {
    auto x = IO::Placeholders{};
    x.contentId = m_contentId;
    x.numberOfInputFrames = m_numberOfInputFrames;
    x.startFrame = m_startFrame;
    return x;
  }

public:
  explicit Application(std::vector<const char *> argv)
      : Common::Application{"MpiEncoder", std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false},
                                {"-f", "Input start frame (e.g. 23)", false}}}
      , m_encoder{json(), json().require("MpiEncoder")}
      , m_contentId{optionValues("-s"sv).front()}
      , m_numberOfInputFrames{std::stoi(optionValues("-n"sv).front())}
      , m_startFrame{std::stoi(optionValues("-f"sv).front())}
      , m_intraPeriod{json().require("intraPeriod").as<int32_t>()}
      , m_inputSequenceConfig{IO::loadSequenceConfig(json(), placeholders(), 0)}
      , m_mpiPcsReader{json(), placeholders(), m_inputSequenceConfig}
      , m_outputBitstreamPath{IO::outputBitstreamPath(json(), placeholders())}
      , m_outputBitstream{m_outputBitstreamPath, std::ios::binary} {
    if (!m_outputBitstream.good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for writing", m_outputBitstreamPath));
    }

    // Support experiments that use a subset of the source cameras
    if (const auto &node = json().optional("inputCameraNames")) {
      Common::logWarning(
          "Source camera names are derived from the sequence configuration. This "
          "functionality to override source camera names is only for internal testing, "
          "e.g. to test with a subset of views.");
      m_inputSequenceConfig.sourceCameraNames = node.asVector<std::string>();
    }

    m_sink = encodeMiv(v3cSampleSink(m_outputBitstream), false);
  }

  void run() override {
    if (1 < m_inputSequenceConfig.sourceViewParams().size()) {
      throw std::runtime_error("Only one input MPI camera is allowed with current version of MPI "
                               "encoder. Please change inputCameraNames field in json !!!");
    }

    m_encoder.setMpiPcsFrameReader(
        [&](int32_t frameIdx) -> MpiPcs::Frame { return m_mpiPcsReader.read(frameIdx); });

    m_encoder.prepareSequence(m_inputSequenceConfig);

    for (int32_t i = 0; i < m_numberOfInputFrames; i += m_intraPeriod) {
      int32_t lastFrame = std::min(m_numberOfInputFrames, i + m_intraPeriod);
      encodeIntraPeriod(i, lastFrame);
    }

    m_sink(std::nullopt);
    reportSummary(m_outputBitstream.tellp());
  }

private:
  void encodeIntraPeriod(int32_t firstFrame, int32_t lastFrame) {
    Common::logInfo("Access unit: [{}, {})", firstFrame, lastFrame);
    m_sink(m_encoder.processAccessUnit(firstFrame, lastFrame));
    popAtlases(firstFrame, lastFrame);
  }

  void popAtlases(int32_t firstFrame, int32_t lastFrame) {
    for (int32_t frameIdx = firstFrame; frameIdx < lastFrame; ++frameIdx) {
      const auto frame = m_encoder.popAtlas();

      auto metadata = Common::Json::Array{};

      for (size_t atlasIdx = 0; atlasIdx < frame.size(); ++atlasIdx) {
        const auto atlasId = MivBitstream::AtlasId{atlasIdx};
        metadata.emplace_back(
            IO::saveOutOfBandVideoFrame(json(), placeholders(), frame[atlasIdx].texture,
                                        MivBitstream::V3cUnitHeader::avd(m_vpsId, atlasId, 0),
                                        frameIdx, MivBitstream::AiAttributeTypeId::ATTR_TEXTURE));
        metadata.emplace_back(IO::saveOutOfBandVideoFrame(
            json(), placeholders(), yuv420(frame[atlasIdx].transparency),
            MivBitstream::V3cUnitHeader::avd(m_vpsId, atlasId, 1), frameIdx,
            MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY));
      }
      if (frameIdx == 0) {
        IO::saveOutOfBandMetadata(json(), placeholders(), metadata);
      }
    }
  }

  void reportSummary(std::streampos bytesWritten) const {
    Common::logInfo("Maximum luma samples per frame is {}", m_encoder.maxLumaSamplesPerFrame());
    Common::logInfo("Total size is {} B ({} kb)", bytesWritten,
                    8e-3 * static_cast<double>(bytesWritten));
    Common::logInfo("Frame count is {}", m_numberOfInputFrames);
    Common::logInfo("Frame rate is {} Hz", m_inputSequenceConfig.frameRate);
    Common::logInfo("Total bitrate is {} kbps", 8e-3 * static_cast<double>(bytesWritten) *
                                                    m_inputSequenceConfig.frameRate /
                                                    m_numberOfInputFrames);
  }
};
} // namespace TMIV::Encoder

auto main(int argc, char *argv[]) -> int32_t {
  try {
    TMIV::Encoder::registerComponents();
    TMIV::Encoder::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (...) {
    return TMIV::Common::handleException();
  }
}
