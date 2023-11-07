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
#include <TMIV/MivBitstream/Formatters.h>
#include <TMIV/MpiPcs/MpiPcs.h>

#include <fstream>

using namespace std::string_view_literals;

namespace TMIV::MpiPcs {
const std::string outputTexturePathFmt = "outputTexturePathFmt";
const std::string outputTransparencyPathFmt = "outputTransparencyPathFmt";

class Application : public Common::Application {
private:
  enum class ConversionMode { None, RawToPcs, PcsToRaw };

  const std::string &m_contentId;
  int32_t m_numberOfInputFrames;
  int32_t m_startFrame;
  MivBitstream::SequenceConfig m_inputSequenceConfig;
  ConversionMode m_conversionMode{ConversionMode::None};

  [[nodiscard]] auto placeholders() const {
    auto x = IO::Placeholders{};
    x.contentId = m_contentId;
    x.numberOfInputFrames = m_numberOfInputFrames;
    x.startFrame = m_startFrame;
    return x;
  }

public:
  explicit Application(std::vector<const char *> argv)
      : Common::Application{"MpiPcs",
                            std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false},
                                {"-f", "Input start frame (e.g. 23)", false},
                                {"-x", "Conversion mode (raw2pcs or pcs2raw)", false}},
                            {}}
      , m_contentId{optionValues("-s"sv).front()}
      , m_numberOfInputFrames{std::stoi(optionValues("-n"sv).front())}
      , m_startFrame{std::stoi(optionValues("-f"sv).front())}
      , m_inputSequenceConfig{IO::loadSequenceConfig(json(), placeholders(), 0)} {
    const auto &conversionModeOption = optionValues("-x"sv).front();

    if (conversionModeOption == "raw2pcs") {
      m_conversionMode = ConversionMode::RawToPcs;
    } else if (conversionModeOption == "pcs2raw") {
      m_conversionMode = ConversionMode::PcsToRaw;
    } else {
      throw std::runtime_error("Invalid conversion options (expected raw2pcs or pcs2raw)");
    }
  }
  void run() override {
    json().checkForUnusedKeys();

    switch (m_conversionMode) {
    case ConversionMode::RawToPcs:
      doRawToPcsConversion();
      break;
    case ConversionMode::PcsToRaw:
      doPcsToRawConversion();
      break;
    default:;
    }
  }

private:
  void doRawToPcsConversion() const {
    const auto cameraName = m_inputSequenceConfig.sourceCameraNames[0];
    const auto cameraConfig = m_inputSequenceConfig.cameraByName(cameraName);
    const auto &viewParams = cameraConfig.viewParams;
    const auto &viewSize = viewParams.ci.projectionPlaneSize();

    MpiPcs::Writer mpiPcsWriter{json(), placeholders(), m_inputSequenceConfig};

    Common::logInfo("RAW to PCS conversion: {} frames ({} layers)", m_numberOfInputFrames,
                    viewParams.nbMpiLayers);

    Common::logInfo("MpiPcs output file: {}", mpiPcsWriter.getPath());

    using geometryValue = MpiPcs::Attribute::GeometryValue;
    const auto layerCount = Common::verifyDownCast<geometryValue>(viewParams.nbMpiLayers);

    for (int32_t frameIdx = 0; frameIdx < m_numberOfInputFrames; ++frameIdx) {
      MpiPcs::Frame mpiPcsFrame{viewSize};

      for (geometryValue layerId = 0; layerId < layerCount; ++layerId) {
        mpiPcsFrame.appendLayer(
            layerId, {yuv420(loadMpiTextureMpiLayer(json(), placeholders(), m_inputSequenceConfig,
                                                    frameIdx, layerId, viewParams.nbMpiLayers)),
                      Common::elementCast<uint8_t>(yuv400(loadMpiTransparencyMpiLayer(
                          json(), placeholders(), m_inputSequenceConfig, frameIdx, layerId,
                          viewParams.nbMpiLayers)))});
      }

      mpiPcsWriter.append(mpiPcsFrame);

      Common::logInfo("Frame #{} transcoded", frameIdx);
    }

    Common::logInfo("RAW to PCS conversion completed");
  }

  void doPcsToRawConversion() const {
    const auto cameraName = m_inputSequenceConfig.sourceCameraNames[0];
    const auto cameraConfig = m_inputSequenceConfig.cameraByName(cameraName);
    const auto &viewParams = cameraConfig.viewParams;
    const auto &viewSize = viewParams.ci.projectionPlaneSize();

    MpiPcs::Reader mpiPcsReader{json(), placeholders(), m_inputSequenceConfig};

    const auto outputDir = json().require("outputDirectory").as<std::filesystem::path>();

    const auto textureVideoFormat =
        IO::videoFormatString(cameraConfig.colorFormatTexture, cameraConfig.bitDepthTexture);

    auto texturePath =
        outputDir /
        fmt::format(fmt::runtime(json().require(outputTexturePathFmt).as<std::string>()),
                    placeholders().numberOfInputFrames, placeholders().contentId,
                    placeholders().testId, cameraName, viewSize.x(), viewSize.y(),
                    textureVideoFormat);

    const auto transparencyVideoFormat = IO::videoFormatString(cameraConfig.colorFormatTransparency,
                                                               cameraConfig.bitDepthTransparency);

    auto transparencyPath =
        outputDir /
        fmt::format(fmt::runtime(json().require(outputTransparencyPathFmt).as<std::string>()),
                    placeholders().numberOfInputFrames, placeholders().contentId,
                    placeholders().testId, cameraName, viewSize.x(), viewSize.y(),
                    transparencyVideoFormat);

    create_directories(texturePath.parent_path());
    create_directories(transparencyPath.parent_path());

    std::ofstream textureStream{texturePath, std::ofstream::binary};
    if (!textureStream.good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for writing", texturePath));
    }

    std::ofstream transparencyStream{transparencyPath, std::ofstream::binary};
    if (!transparencyStream.good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for writing", transparencyPath));
    }

    Common::logInfo("PCS to RAW conversion: {} frames ({} layers)", m_numberOfInputFrames,
                    viewParams.nbMpiLayers);

    Common::logInfo("Texture output file {}", texturePath);
    Common::logInfo("Transparency output file {}", transparencyPath);

    using geometryValue = MpiPcs::Attribute::GeometryValue;
    const auto layerCount = Common::verifyDownCast<geometryValue>(viewParams.nbMpiLayers);

    for (int32_t frameIdx = 0; frameIdx < m_numberOfInputFrames; ++frameIdx) {
      const auto mpiPcsFrame = mpiPcsReader.read(frameIdx);

      for (geometryValue layerId = 0; layerId < layerCount; ++layerId) {
        const auto [textureLayer, transparencyLayer] = mpiPcsFrame.getLayer(layerId);

        textureLayer.writeTo(textureStream);
        if (!textureStream.good()) {
          throw std::runtime_error(fmt::format("Failed to write to {}", texturePath));
        }

        transparencyLayer.writeTo(transparencyStream);

        if (!transparencyStream.good()) {
          throw std::runtime_error(fmt::format("Failed to write to {}", transparencyPath));
        }
      }

      Common::logInfo("Frame #{} transcoded", frameIdx);
    }

    Common::logInfo("PCS to RAW conversion completed");
  }
};

} // namespace TMIV::MpiPcs

auto main(int argc, char *argv[]) -> int32_t {
  try {
    TMIV::MpiPcs::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (...) {
    return TMIV::Common::handleException();
  }
}