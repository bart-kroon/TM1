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

#include <TMIV/Encoder/Encoder.h>

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Encoder/EncodeMiv.h>
#include <TMIV/Encoder/V3cSampleSink.h>
#include <TMIV/IO/IO.h>
#include <TMIV/MivBitstream/Formatters.h>

#include <fstream>

using namespace std::string_view_literals;

namespace TMIV::Encoder {
void registerComponents();

class Application : public Common::Application {
private:
  Encoder m_encoder;
  const std::string &m_contentId;
  int32_t m_numberOfInputFrames;
  int32_t m_startFrame;
  int32_t m_interPeriod{};
  int32_t m_intraPeriod;
  static constexpr uint8_t m_vpsId = 0;

  MivBitstream::SequenceConfig m_inputSequenceConfig;
  std::filesystem::path m_outputBitstreamPath;
  std::ofstream m_outputBitstream;
  Common::Sink<EncoderParams> m_sink;

  [[nodiscard]] auto placeholders() const {
    auto x = IO::Placeholders{};
    x.contentId = m_contentId;
    x.numberOfInputFrames = m_numberOfInputFrames;
    x.startFrame = m_startFrame;
    return x;
  }

public:
  explicit Application(std::vector<const char *> argv)
      : Common::Application{"Encoder", std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false},
                                {"-f", "Input start frame (e.g. 23)", false}}}
      , m_encoder{json()}
      , m_contentId{optionValues("-s"sv).front()}
      , m_numberOfInputFrames{std::stoi(optionValues("-n"sv).front())}
      , m_startFrame{std::stoi(optionValues("-f"sv).front())}
      , m_intraPeriod{json().require("intraPeriod").as<int32_t>()}
      , m_inputSequenceConfig{IO::loadSequenceConfig(json(), placeholders(), 0)}
      , m_outputBitstreamPath{IO::outputBitstreamPath(json(), placeholders())}
      , m_outputBitstream{m_outputBitstreamPath, std::ios::binary}
      , m_sink{encodeMiv(v3cSampleSink(m_outputBitstream),
                         json().require("rewriteParameterSets").as<bool>())} {
    if (!m_outputBitstream.good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for writing", m_outputBitstreamPath));
    }
    if (const auto &node = json().optional("interPeriod")) {
      m_interPeriod = node.as<int32_t>();
    } else {
      m_interPeriod = m_intraPeriod;
    }

    VERIFY(1 <= m_interPeriod && m_intraPeriod % m_interPeriod == 0);

    // Support experiments that use a subset of the source cameras
    if (const auto &node = json().optional("inputCameraNames")) {
      Common::logWarning(
          "Source camera names are derived from the sequence configuration. This "
          "functionality to override source camera names is only for internal testing, "
          "e.g. to test with a subset of views.");
      m_inputSequenceConfig.sourceCameraNames = node.asVector<std::string>();
    }
    if (const auto &node = json().optional("sourceCameraIds")) {
      m_inputSequenceConfig.sourceCameraIds = node.asVector<uint16_t>();
    }
  }

  void run() override {
    m_encoder.prepareSequence(
        m_inputSequenceConfig,
        IO::loadMultiviewFrame(json(), placeholders(), m_inputSequenceConfig, 0));

    for (int32_t i = 0; i < m_numberOfInputFrames; i += m_interPeriod) {
      int32_t lastFrame = std::min(m_numberOfInputFrames, i + m_interPeriod);
      encodeInterPeriod(i, lastFrame);
    }

    m_sink(std::nullopt);
    reportSummary(m_outputBitstream.tellp());
  }

private:
  void encodeInterPeriod(int32_t firstFrame, int32_t lastFrame) {
    Common::logInfo("Inter period: [{}, {})", firstFrame, lastFrame);
    m_encoder.prepareAccessUnit();
    pushFrames(firstFrame, lastFrame);
    m_sink(m_encoder.completeAccessUnit());
    popAtlases(firstFrame, lastFrame);
  }

  void pushFrames(int32_t firstFrame, int32_t lastFrame) {
    for (int32_t i = firstFrame; i < lastFrame; ++i) {
      m_encoder.pushFrame(IO::loadMultiviewFrame(json(), placeholders(), m_inputSequenceConfig, i));
    }
  }

  void popAtlases(int32_t firstFrame, int32_t lastFrame) {
    for (int32_t frameIdx = firstFrame; frameIdx < lastFrame; ++frameIdx) {
      const auto frame = m_encoder.popAtlas();

      auto metadata = Common::Json::Array{};

      for (size_t atlasIdx = 0; atlasIdx < frame.size(); ++atlasIdx) {
        const auto sub = saveAtlasFrame(MivBitstream::AtlasId{atlasIdx}, frameIdx, frame[atlasIdx]);
        metadata.insert(metadata.end(), sub.cbegin(), sub.cend());
      }
      if (frameIdx == 0) {
        IO::saveOutOfBandMetadata(json(), placeholders(), metadata);
      }
    }
  }

  auto saveAtlasFrame(MivBitstream::AtlasId atlasId, int32_t frameIdx,
                      const Common::V3cFrame &frame) -> Common::Json::Array {
    using VUH = MivBitstream::V3cUnitHeader;
    using VUT = MivBitstream::VuhUnitType;
    using ATI = MivBitstream::AiAttributeTypeId;

    auto metadata = Common::Json::Array{};
    uint8_t attrIdx{};

    const auto save = [this, frameIdx, &metadata,
                       &attrIdx](const Common::Frame<> &component, VUH vuh,
                                 ATI attrTypeId = ATI::ATTR_UNSPECIFIED) {
      if (!component.empty()) {
        metadata.emplace_back(IO::saveOutOfBandVideoFrame(json(), placeholders(), yuv420(component),
                                                          vuh, frameIdx, attrTypeId));
        if (vuh.vuh_unit_type() == VUT::V3C_AVD) {
          ++attrIdx;
        }
      }
    };

    save(frame.occupancy, VUH::ovd(m_vpsId, atlasId));
    save(frame.geometry, VUH::gvd(m_vpsId, atlasId));
    save(frame.texture, VUH::avd(m_vpsId, atlasId, attrIdx), ATI::ATTR_TEXTURE);
    save(frame.transparency, VUH::avd(m_vpsId, atlasId, attrIdx), ATI::ATTR_TRANSPARENCY);
    save(frame.packed, VUH::pvd(m_vpsId, atlasId));
    return metadata;
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
