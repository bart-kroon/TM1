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
#include <TMIV/Common/Factory.h>
#include <TMIV/MpiPcs/MpiPcs.h>

#include <fstream>
#include <iostream>

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
      : Common::Application{"MpiPcs", std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false},
                                {"-f", "Input start frame (e.g. 23)", false},
                                {"-x", "Conversion mode (raw2pcs or pcs2raw)", false}}}
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

    fmt::print("RAW to PCS conversion: {} frames ({} layers)\n", m_numberOfInputFrames,
               viewParams.nbMpiLayers);

    fmt::print("MpiPcs output file: {}\n", mpiPcsWriter.getPath());

    using geometryValue = MpiPcs::Attribute::GeometryValue;
    const auto layerCount = Common::verifyDownCast<geometryValue>(viewParams.nbMpiLayers);

    for (int frameId = 0; frameId < m_numberOfInputFrames; ++frameId) {
      MpiPcs::Frame mpiPcsFrame{viewSize};

      for (geometryValue layerId = 0; layerId < layerCount; ++layerId) {
        auto textureLayer = loadMpiTextureMpiLayer(json(), placeholders(), m_inputSequenceConfig,
                                                   frameId, layerId, viewParams.nbMpiLayers);

        auto transparencyLayer =
            loadMpiTransparencyMpiLayer(json(), placeholders(), m_inputSequenceConfig, frameId,
                                        layerId, viewParams.nbMpiLayers);

        mpiPcsFrame.appendLayer(layerId, {std::move(textureLayer), std::move(transparencyLayer)});
      }

      mpiPcsWriter.append(mpiPcsFrame);

      fmt::print("Frame #{} transcoded\n", frameId);
    }

    fmt::print("RAW to PCS conversion completed\n");
  }

  void doPcsToRawConversion() const {
    const auto cameraName = m_inputSequenceConfig.sourceCameraNames[0];
    const auto cameraConfig = m_inputSequenceConfig.cameraByName(cameraName);
    const auto &viewParams = cameraConfig.viewParams;
    const auto &viewSize = viewParams.ci.projectionPlaneSize();

    MpiPcs::Reader mpiPcsReader{json(), placeholders(), m_inputSequenceConfig};

    const auto outputDir = json().require(IO::outputDirectory).as<std::filesystem::path>();

    auto texturePath =
        outputDir / fmt::format(json().require(outputTexturePathFmt).as<std::string>(),
                                placeholders().numberOfInputFrames, placeholders().contentId,
                                placeholders().testId, cameraName, viewSize.x(), viewSize.y(),
                                "yuv420p10le");

    auto transparencyPath =
        outputDir / fmt::format(json().require(outputTransparencyPathFmt).as<std::string>(),
                                placeholders().numberOfInputFrames, placeholders().contentId,
                                placeholders().testId, cameraName, viewSize.x(), viewSize.y(),
                                "yuv420p");

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

    fmt::print("PCS to RAW conversion: {} frames ({} layers)\n", m_numberOfInputFrames,
               viewParams.nbMpiLayers);

    fmt::print("Texture output file {}\n", texturePath);
    fmt::print("Transparency output file {}\n", transparencyPath);

    using geometryValue = MpiPcs::Attribute::GeometryValue;
    const auto layerCount = Common::verifyDownCast<geometryValue>(viewParams.nbMpiLayers);

    for (int frameId = 0; frameId < m_numberOfInputFrames; ++frameId) {
      const auto mpiPcsFrame = mpiPcsReader.read(frameId);

      for (geometryValue layerId = 0; layerId < layerCount; ++layerId) {
        const auto [textureLayer, transparencyLayer] = mpiPcsFrame.getLayer(layerId);

        textureLayer.dump(textureStream);
        if (!textureStream.good()) {
          throw std::runtime_error(fmt::format("Failed to write to {}", texturePath));
        }

        transparencyLayer.dump(transparencyStream);
        Common::padChroma<Common::YUV400P8>(transparencyStream,
                                            transparencyLayer.getDiskSize() -
                                                transparencyLayer.getMemorySize());
        if (!transparencyStream.good()) {
          throw std::runtime_error(fmt::format("Failed to write to {}", transparencyPath));
        }
      }

      fmt::print("Frame #{} transcoded\n", frameId);
    }

    fmt::print("PCS to RAW conversion completed\n");
  }
};

} // namespace TMIV::MpiPcs

auto main(int argc, char *argv[]) -> int {
  try {
    TMIV::MpiPcs::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}