/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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
#include <TMIV/FramePacker/FramePackerStage.h>
#include <TMIV/MivBitstream/Formatters.h>
#include <TMIV/MpiPcs/MpiPcs.h>
#include <TMIV/Packer/Packer.h>

#include "CodableUnitEncoder.h"

#include <fstream>

using namespace std::string_view_literals;

namespace TMIV::Encoder {
class Application : public Common::Application {
public:
  explicit Application(std::vector<const char *> argv)
      : Common::Application{"MpiEncoder", std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false},
                                {"-f", "Input start frame (e.g. 23)", false}}}
      , m_placeholders{placeholders()}
      , m_sequenceConfig{IO::loadSequenceConfig(json(), placeholders(), 0)}
      , m_mpiPcsReader{json(), placeholders(), m_sequenceConfig}
      , m_encoder{json()}
      , m_framePacker{json()}
      , m_codableUnitEncoder{json(), placeholders()} {
    m_encoder.source.connectTo(m_framePacker);
    m_framePacker.source.connectTo(m_codableUnitEncoder);
  }

  void run() override {
    json().checkForUnusedKeys();

    if (1 < m_sequenceConfig.sourceViewParams().size()) {
      throw std::runtime_error("Only one input MPI camera is allowed with current version of MPI "
                               "encoder. Please change inputCameraNames field in json !!!");
    }

    for (int32_t i = 0; i < m_placeholders.numberOfInputFrames; ++i) {
      m_encoder.encode({m_sequenceConfig, m_mpiPcsReader.read(i)});
    }

    m_encoder.flush();

    reportSummary(m_codableUnitEncoder.bytesWritten());
  }

private:
  [[nodiscard]] auto placeholders() const -> IO::Placeholders {
    auto x = IO::Placeholders{};
    x.contentId = optionValues("-s"sv).front();
    x.numberOfInputFrames = std::stoi(optionValues("-n"sv).front());
    x.startFrame = std::stoi(optionValues("-f"sv).front());
    return x;
  }

  void reportSummary(std::streampos bytesWritten) const {
    Common::logInfo("Maximum luma samples per frame is {}", m_encoder.maxLumaSamplesPerFrame());
    Common::logInfo("Total size is {} B ({} kb)", bytesWritten,
                    8e-3 * static_cast<double>(bytesWritten));
    Common::logInfo("Frame count is {}", m_placeholders.numberOfInputFrames);
    Common::logInfo("Frame rate is {} Hz", m_sequenceConfig.frameRate);
    Common::logInfo("Total bitrate is {} kbps", 8e-3 * static_cast<double>(bytesWritten) *
                                                    m_sequenceConfig.frameRate /
                                                    m_placeholders.numberOfInputFrames);
  }

  IO::Placeholders m_placeholders;
  MivBitstream::SequenceConfig m_sequenceConfig;
  MpiPcs::Reader m_mpiPcsReader;
  MpiEncoder m_encoder;
  FramePacker::FramePackerStage m_framePacker;
  CodableUnitEncoder m_codableUnitEncoder;
};

void registerComponents() {
  auto &packers = Common::Factory<Packer::IPacker>::getInstance();
  packers.registerAs<Packer::Packer>("Packer");
}
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
