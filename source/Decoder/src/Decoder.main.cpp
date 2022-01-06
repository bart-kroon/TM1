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
#include <TMIV/Decoder/MivDecoder.h>
#include <TMIV/Decoder/OutputLog.h>
#include <TMIV/Decoder/PreRenderer.h>
#include <TMIV/Decoder/V3cSampleStreamDecoder.h>
#include <TMIV/IO/IO.h>
#include <TMIV/MivBitstream/SequenceConfig.h>
#include <TMIV/Renderer/Front/MultipleFrameRenderer.h>
#include <TMIV/Renderer/Front/mapInputToOutputFrames.h>
#include <TMIV/Renderer/RecoverPrunedViews.h>

#include <fstream>
#include <iostream>
#include <memory>

using namespace std::string_view_literals;
using namespace std::string_literals;

namespace TMIV::Decoder {
void registerComponents();

class Application : public Common::Application {
private:
  PreRenderer m_preRenderer;
  IO::Placeholders m_placeholders;
  Renderer::Front::MultipleFrameRenderer m_renderer;
  std::multimap<int32_t, int32_t> m_inputToOutputFrameIdMap;
  std::filesystem::path m_inputBitstreamPath;
  std::ifstream m_inputBitstream;
  V3cSampleStreamDecoder m_vssDecoder;
  MivDecoder m_mivDecoder;
  MivBitstream::SequenceConfig m_outputSequenceConfig;
  std::ofstream m_outputLog;

public:
  explicit Application(std::vector<const char *> argv)
      : Common::Application{"Decoder", std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false},
                                {"-N", "Number of output frames (e.g. 300)", false},
                                {"-r", "Test point (e.g. QP3 or R0)", false},
                                {"-v", "Source view to render (e.g. v11)", true},
                                {"-P", "Pose trace to render (e.g. p02)", true}}}
      , m_preRenderer{json()}
      , m_placeholders{optionValues("-s").front(), optionValues("-r").front(),
                       std::stoi(optionValues("-n"sv).front()),
                       std::stoi(optionValues("-N"sv).front())}
      , m_renderer{json(), optionValues("-v"), optionValues("-P"), m_placeholders}
      , m_inputToOutputFrameIdMap{Renderer::Front::mapInputToOutputFrames(
            m_placeholders.numberOfInputFrames, m_placeholders.numberOfOutputFrames)}
      , m_inputBitstreamPath{IO::inputBitstreamPath(json(), m_placeholders)}
      , m_inputBitstream{m_inputBitstreamPath, std::ios::binary}
      , m_vssDecoder{createVssDecoder()}
      , m_mivDecoder{[this]() { return m_vssDecoder(); }} {
    setFrameServer(m_mivDecoder);
    tryOpenOutputLog();
  }

  void run() override {
    while (auto frame = m_mivDecoder()) {
      if (m_outputLog.is_open()) {
        writeFrameToOutputLog(*frame, m_outputLog);
      }

      // Check which frames to render if we would
      const auto range = m_inputToOutputFrameIdMap.equal_range(frame->foc);
      if (range.first == range.second) {
        return;
      }

      // Recover geometry, occupancy, and filter blockToPatchMap
      m_preRenderer.preRenderFrame(*frame);

      outputSequenceConfig(frame->sequenceConfig(), frame->foc);
      IO::optionalSaveBlockToPatchMaps(json(), m_placeholders, frame->foc, *frame);
      optionalSavePrunedFrame(frame->foc, Renderer::recoverPrunedViews(*frame));

      m_renderer.renderMultipleFrames(*frame, range.first, range.second);
    }
  }

private:
  auto createVssDecoder() -> V3cSampleStreamDecoder {
    if (!m_inputBitstream.good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for reading", m_inputBitstreamPath));
    }
    return V3cSampleStreamDecoder{m_inputBitstream};
  }

  void setFrameServer(MivDecoder &mivDecoder) const {
    mivDecoder.setFrameServer([this](MivBitstream::V3cUnitHeader vuh, int32_t frameIdx,
                                     const MivBitstream::V3cParameterSet &vps,
                                     const MivBitstream::AtlasSequenceParameterSetRBSP &asps) {
      if (frameIdx < m_placeholders.numberOfInputFrames) {
        return IO::loadOutOfBandVideoFrame(json(), m_placeholders, vuh, frameIdx, vps, asps);
      }
      return Common::Frame<>{};
    });
  }

  void tryOpenOutputLog() {
    if (const auto &node = json().optional("outputLogPath")) {
      const auto outputLogPath = node.as<std::string>();
      m_outputLog.open(outputLogPath, std::ios::out | std::ios::binary);

      if (!m_outputLog) {
        throw std::runtime_error(
            fmt::format("Failed to open output log \"{}\" for writing", outputLogPath));
      }
    }
  }

  void outputSequenceConfig(MivBitstream::SequenceConfig sc, int32_t foc) {
    // NOTE(#463): Inject the frame count into the sequence configuration, because the decoder
    // library does not have that knowledge.
    sc.numberOfFrames = std::stoi(optionValues("-n"sv).front());

    if (m_outputSequenceConfig != sc) {
      m_outputSequenceConfig = std::move(sc);
      IO::optionalSaveSequenceConfig(json(), m_placeholders, foc, m_outputSequenceConfig);
    }
  }

  void optionalSavePrunedFrame(int32_t frameIdx, const Common::V3cFrameList &frame) const {
    uint16_t viewIdx{};

    for (const auto &view : frame) {
      IO::optionalSavePrunedFrame(json(), m_placeholders, view.occupancy,
                                  MivBitstream::VuhUnitType::V3C_OVD, {frameIdx, viewIdx});

      IO::optionalSavePrunedFrame(json(), m_placeholders, view.geometry,
                                  MivBitstream::VuhUnitType::V3C_GVD, {frameIdx, viewIdx});

      IO::optionalSavePrunedFrame(json(), m_placeholders, yuv420(view.texture),
                                  MivBitstream::VuhUnitType::V3C_AVD, {frameIdx, viewIdx},
                                  MivBitstream::AiAttributeTypeId::ATTR_TEXTURE);

      IO::optionalSavePrunedFrame(json(), m_placeholders, view.transparency,
                                  MivBitstream::VuhUnitType::V3C_AVD, {frameIdx, viewIdx},
                                  MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY);

      ++viewIdx;
    }
  }
};
} // namespace TMIV::Decoder

auto main(int argc, char *argv[]) -> int32_t {
  try {
    TMIV::Decoder::registerComponents();
    TMIV::Decoder::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  } catch (std::logic_error &e) {
    std::cerr << e.what() << std::endl;
    return 3;
  }
}
