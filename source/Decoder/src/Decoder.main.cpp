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
#include <TMIV/Decoder/DecodeAtlasSubBitstream.h>
#include <TMIV/Decoder/DecodeMiv.h>
#include <TMIV/Decoder/DecodeNalUnitStream.h>
#include <TMIV/Decoder/DecodeV3cSampleStream.h>
#include <TMIV/Decoder/DecodeVideoSubBitstream.h>
#include <TMIV/Decoder/OutputLog.h>
#include <TMIV/Decoder/PreRenderer.h>
#include <TMIV/IO/IO.h>
#include <TMIV/MivBitstream/Formatters.h>
#include <TMIV/MivBitstream/SequenceConfig.h>
#include <TMIV/PtlChecker/PtlChecker.h>
#include <TMIV/Renderer/Front/MultipleFrameRenderer.h>
#include <TMIV/Renderer/Front/mapInputToOutputFrames.h>
#include <TMIV/Renderer/RecoverPrunedViews.h>
#include <TMIV/VideoDecoder/VideoDecoder.h>

#include <fstream>
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
  PtlChecker::SharedChecker m_checker;
  Common::Source<MivBitstream::AccessUnit> m_mivDecoder;
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
      , m_checker{std::make_shared<PtlChecker::PtlChecker>()}
      , m_mivDecoder{decodeMiv()} {
    tryOpenOutputLog();
  }

  void run() override {
    for (int32_t inputFrameIdx = 0; inputFrameIdx < m_placeholders.numberOfInputFrames;
         ++inputFrameIdx) {
      if (auto frame = m_mivDecoder()) {
        VERIFY_MIVBITSTREAM(frame->frameIdx == inputFrameIdx);

        if (m_outputLog.is_open()) {
          writeFrameToOutputLog(*frame, m_outputLog);
        }

        // Recover geometry, occupancy, and filter blockToPatchMap
        m_preRenderer.preRenderFrame(*frame);

        outputSequenceConfig(frame->sequenceConfig(), frame->frameIdx);
        IO::optionalSaveBlockToPatchMaps(json(), m_placeholders, frame->frameIdx, *frame);
        optionalSavePrunedFrame(frame->frameIdx, Renderer::recoverPrunedViews(*frame));

        // Render multiple frames
        const auto range = m_inputToOutputFrameIdMap.equal_range(frame->frameIdx);
        m_renderer.renderMultipleFrames(*frame, range.first, range.second);
      } else {
        throw std::runtime_error(
            fmt::format("The input frame count was set to {} but the bitstream only has {} frames.",
                        m_placeholders.numberOfInputFrames, inputFrameIdx));
      }
    }
  }

private:
  auto decodeMiv() -> Common::Source<MivBitstream::AccessUnit> {
    if (!m_inputBitstream.good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for reading", m_inputBitstreamPath));
    }
    return Decoder::decodeMiv(decodeV3cSampleStream(m_inputBitstream), videoDecoderFactory(),
                              m_checker, commonAtlasDecoderFactory(), atlasDecoderFactory());
  }

  auto videoDecoderFactory() -> VideoDecoderFactory {
    return [this](Common::Source<MivBitstream::VideoSubBitstream> source,
                  const MivBitstream::V3cParameterSet &vps,
                  MivBitstream::V3cUnitHeader vuh) -> Common::Source<Common::DecodedFrame> {
      source = Common::test(source);

      if (source) {
        return decodeVideo(vps, decodeNalUnitStream(decodeVideoSubBitstream(std::move(source))));
      }
      return loadOutOfBandVideo(vuh);
    };
  }

  static auto decodeVideo(const MivBitstream::V3cParameterSet &vps,
                          Common::Source<std::string> source)
      -> Common::Source<Common::DecodedFrame> {
    const auto codecGroupIdc = vps.profile_tier_level().ptl_profile_codec_group_idc();

    auto result = [codecGroupIdc, &source]() {
      switch (codecGroupIdc) {
      case MivBitstream::PtlProfileCodecGroupIdc::AVC_Progressive_High:
        return VideoDecoder::decodeAvcProgressiveHigh(std::move(source));
      case MivBitstream::PtlProfileCodecGroupIdc::HEVC_Main10:
        return VideoDecoder::decodeHevcMain10(std::move(source));
      case MivBitstream::PtlProfileCodecGroupIdc::HEVC444:
        return VideoDecoder::decodeHevc444(std::move(source));
      case MivBitstream::PtlProfileCodecGroupIdc::VVC_Main10:
        return VideoDecoder::decodeVvcMain10(std::move(source));
      default:
        return Common::Source<Common::DecodedFrame>{};
      }
    }();

    if (result) {
      return result;
    }
    throw std::runtime_error(
        fmt::format("Failed to initialize a video decoder for codec group IDC {}", codecGroupIdc));
  }

  auto loadOutOfBandVideo(MivBitstream::V3cUnitHeader vuh) -> Common::Source<Common::DecodedFrame> {
    return [this, vuh, frameIdx = int32_t{}]() mutable -> std::optional<Common::DecodedFrame> {
      if (frameIdx < m_placeholders.numberOfInputFrames) {
        return IO::loadOutOfBandVideoFrame(json(), m_placeholders, vuh, frameIdx++);
      }
      return std::nullopt;
    };
  }

  auto commonAtlasDecoderFactory() -> CommonAtlasDecoderFactory {
    return [checker = m_checker](Common::Source<MivBitstream::AtlasSubBitstream> source)
               -> Common::Source<CommonAtlasAccessUnit> {
      return decodeCommonAtlas(decodeAtlasSubBitstream(std::move(source)), checker);
    };
  }

  auto atlasDecoderFactory() -> AtlasDecoderFactory {
    return
        [checker = m_checker](Common::Source<MivBitstream::AtlasSubBitstream> source,
                              MivBitstream::V3cUnitHeader vuh) -> Common::Source<AtlasAccessUnit> {
          return decodeAtlas(decodeAtlasSubBitstream(std::move(source)), vuh, checker);
        };
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

  void outputSequenceConfig(MivBitstream::SequenceConfig sc, int32_t frameIdx) {
    // NOTE(#463): Inject the frame count into the sequence configuration, because the decoder
    // library does not have that knowledge.
    sc.numberOfFrames = std::stoi(optionValues("-n"sv).front());

    if (m_outputSequenceConfig != sc) {
      m_outputSequenceConfig = std::move(sc);
      IO::optionalSaveSequenceConfig(json(), m_placeholders, frameIdx, m_outputSequenceConfig);
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
  } catch (...) {
    return TMIV::Common::handleException();
  }
}
