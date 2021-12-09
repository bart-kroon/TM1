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

#include <TMIV/IO/IO.h>

#include <fstream>
#include <regex>

using namespace std::string_literals;

namespace TMIV::IO {
const std::string configDirectory = "configDirectory";
const std::string inputBitstreamPathFmt = "inputBitstreamPathFmt";
const std::string inputCameraNames = "inputCameraNames";
const std::string inputDirectory = "inputDirectory";
const std::string inputEntityPathFmt = "inputEntityPathFmt";
const std::string inputGeometryPathFmt = "inputGeometryPathFmt";
const std::string inputGeometryVideoFramePathFmt = "inputGeometryVideoFramePathFmt"s;
const std::string inputGeometryVsbPathFmt = "inputGeometryVideoSubBitstreamPathFmt";
const std::string inputMaterialIdVsbPathFmt = "inputMaterialIdVideoSubBitstreamPathFmt";
const std::string inputNormalVsbPathFmt = "inputNormalVideoSubBitstreamPathFmt";
const std::string inputOccupancyVideoFramePathFmt = "inputOccupancyVideoFramePathFmt";
const std::string inputOccupancyVsbPathFmt = "inputOccupancyVideoSubBitstreamPathFmt";
const std::string inputPoseTracePathFmt = "inputPoseTracePathFmt";
const std::string inputReflectanceVsbPathFmt = "inputReflectanceVideoSubBitstreamPathFmt";
const std::string inputSequenceConfigPathFmt = "inputSequenceConfigPathFmt";
const std::string inputTexturePathFmt = "inputTexturePathFmt";
const std::string inputTextureVideoFramePathFmt = "inputTextureVideoFramePathFmt";
const std::string inputTextureVsbPathFmt = "inputTextureVideoSubBitstreamPathFmt";
const std::string inputTransparencyPathFmt = "inputTransparencyPathFmt";
const std::string inputTransparencyVideoFramePathFmt = "inputTransparencyVideoFramePathFmt";
const std::string inputTransparencyVsbPathFmt = "inputTransparencyVideoSubBitstreamPathFmt";
const std::string inputPackedVideoFramePathFmt = "inputPackedVideoFramePathFmt";
const std::string inputViewportParamsPathFmt = "inputViewportParamsPathFmt"s;
const std::string inputPackedVsbPathFmt = "inputPackedVideoSubBitstreamPathFmt";

template <typename Element>
auto loadFrame(const std::filesystem::path &path, int32_t frameIndex, Common::Vec2i frameSize,
               uint32_t bitDepth, Common::ColorFormat colorFormat) -> Common::Frame<Element> {
  return Common::withElement(bitDepth, [&](auto zero) -> Common::Frame<Element> {
    using FileElement = decltype(zero);

    auto frame = Common::Frame<FileElement>{frameSize, bitDepth, colorFormat};

    std::ifstream stream{path, std::ios::binary};
    if (!stream.good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for reading", path));
    }

    stream.seekg(std::streampos(frameIndex) * frame.getDiskSize());
    if (!stream.good()) {
      throw std::runtime_error(
          fmt::format("Failed to seek for reading to frame {} from {}", frameIndex, path));
    }

    frame.read(stream);
    if (!stream.good()) {
      throw std::runtime_error(fmt::format("Failed to read frame {} from {}", frameIndex, path));
    }

    if constexpr (std::is_same_v<FileElement, Element>) {
      return frame;
    } else {
      auto result =
          Common::Frame<Element>{frame.getSize(), frame.getBitDepth(), frame.getColorFormat()};

      for (size_t i = 0; i < result.getNumberOfPlanes(); ++i) {
        std::transform(frame.getPlane(i).cbegin(), frame.getPlane(i).cend(),
                       result.getPlane(i).begin(),
                       [](FileElement value) { return Common::assertDownCast<Element>(value); });
      }

      return result;
    }
  });
}

namespace {
auto requantize(Common::Frame<> frame) {
  const auto maxValue = frame.maxValue();

  std::transform(frame.getPlane(0).cbegin(), frame.getPlane(0).cend(), frame.getPlane(0).begin(),
                 [maxValue](uint32_t x) {
                   const auto y = (0xFFFF * x + maxValue / 2) / maxValue;
                   return static_cast<uint16_t>(y);
                 });

  frame.setBitDepth(16);
  return frame;
}
} // namespace

auto loadMultiviewFrame(const Common::Json &config, const Placeholders &placeholders,
                        const MivBitstream::SequenceConfig &sc, int32_t frameIndex)
    -> Common::MVD16Frame {
  auto frame = Common::MVD16Frame(sc.sourceCameraNames.size());

  const auto inputDir = config.require(inputDirectory).as<std::filesystem::path>();
  const auto startFrame = placeholders.startFrame;
  fmt::print("Loading multiview frame {0} with start frame offset {1} (= {2}).\n", frameIndex,
             startFrame, frameIndex + startFrame);

  for (size_t v = 0; v < frame.size(); ++v) {
    const auto name = sc.sourceCameraNames[v];
    const auto camera = sc.cameraByName(name);
    const auto &vp = camera.viewParams;
    const auto frameSize = vp.ci.projectionPlaneSize();

    if (const auto &node = config.optional(inputTexturePathFmt)) {
      const auto path =
          inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                 placeholders.contentId, placeholders.testId, name, frameSize.x(),
                                 frameSize.y(), camera.textureVideoFormat());
      frame[v].texture = loadFrame<>(path, startFrame + frameIndex, frameSize, camera.bitDepthColor,
                                     Common::ColorFormat::YUV420);
    }

    if (const auto &node = config.optional(inputTransparencyPathFmt)) {
      const auto path =
          inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                 placeholders.contentId, placeholders.testId, name, frameSize.x(),
                                 frameSize.y(), camera.transparencyVideoFormat());
      frame[v].transparency = loadFrame<>(path, startFrame + frameIndex, frameSize,
                                          camera.bitDepthTransparency, Common::ColorFormat::YUV400);
    }

    if (const auto &node = config.optional(inputGeometryPathFmt)) {
      const auto path =
          inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                 placeholders.contentId, placeholders.testId, name, frameSize.x(),
                                 frameSize.y(), camera.geometryVideoFormat());
      frame[v].depth = requantize(loadFrame<>(path, startFrame + frameIndex, frameSize,
                                              camera.bitDepthDepth, Common::ColorFormat::YUV400));
    }

    if (const auto &node = config.optional(inputEntityPathFmt)) {
      const auto path =
          inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                 placeholders.contentId, placeholders.testId, name, frameSize.x(),
                                 frameSize.y(), camera.entitiesVideoFormat());
      frame[v].entities = loadFrame<>(path, startFrame + frameIndex, frameSize,
                                      camera.bitDepthEntities, Common::ColorFormat::YUV400);
    }
  }

  return frame;
}

auto loadMpiTextureMpiLayer(const Common::Json &config, const Placeholders &placeholders,
                            const MivBitstream::SequenceConfig &sc, int32_t frameIndex,
                            int32_t mpiLayerIndex, int32_t nbMpiLayers) -> Common::TextureFrame {
  const auto inputDir = config.require(inputDirectory).as<std::filesystem::path>();
  const auto &node = config.require(inputTexturePathFmt);

  const auto name = sc.sourceCameraNames[0];

  const auto camera = sc.cameraByName(name);
  const auto &vp = camera.viewParams;
  const auto frameSize = vp.ci.projectionPlaneSize();

  const auto path = inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                           placeholders.contentId, placeholders.testId, name,
                                           frameSize.x(), frameSize.y(), "yuv420p10le");

  auto texture = loadFrame<>(path, frameIndex * nbMpiLayers + mpiLayerIndex, frameSize,
                             camera.bitDepthColor, Common::ColorFormat::YUV420);

  return texture;
}

auto loadMpiTransparencyMpiLayer(const Common::Json &config, const Placeholders &placeholders,
                                 const MivBitstream::SequenceConfig &sc, int32_t frameIndex,
                                 int32_t mpiLayerIndex, int32_t nbMpiLayers)
    -> Common::Transparency8Frame {
  const auto inputDir = config.require(inputDirectory).as<std::filesystem::path>();
  const auto &node = config.require(inputTransparencyPathFmt);

  const auto name = sc.sourceCameraNames[0];

  const auto camera = sc.cameraByName(name);
  const auto &vp = camera.viewParams;
  const auto frameSize = vp.ci.projectionPlaneSize();

  const auto path = inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                           placeholders.contentId, placeholders.testId, name,
                                           frameSize.x(), frameSize.y(), "yuv420p");

  return loadFrame<uint8_t>(path, frameIndex * nbMpiLayers + mpiLayerIndex, frameSize,
                            camera.bitDepthTransparency, Common::ColorFormat::YUV400);
}

namespace {
struct Pose {
  Common::Vec3f position;
  Common::Vec3f rotation;
};

auto loadPoseFromCSV(std::istream &stream, int32_t frameIndex) -> Pose {
  std::string line;
  getline(stream, line);

  std::regex re_header(R"(\s*X\s*,\s*Y\s*,\s*Z\s*,\s*Yaw\s*,\s*Pitch\s*,\s*Roll\s*)");
  if (!std::regex_match(line, re_header)) {
    throw std::runtime_error("Format error in the pose trace header");
  }

  int32_t currentFrameIndex = 0;
  std::regex re_row("([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+)");
  std::regex re_empty("\\s*");
  bool trailing_empty_lines = false;

  while (getline(stream, line)) {
    std::smatch match;
    if (!trailing_empty_lines && std::regex_match(line, match, re_row)) {
      if (currentFrameIndex == frameIndex) {
        return {Common::Vec3f({stof(match[1].str()), stof(match[2].str()), stof(match[3].str())}),
                Common::Vec3f({stof(match[4].str()), stof(match[5].str()), stof(match[6].str())})};
      }
      { currentFrameIndex++; }
    } else if (std::regex_match(line, re_empty)) {
      trailing_empty_lines = true;
    } else {
      throw std::runtime_error("Format error in a pose trace row");
    }
  }

  throw std::runtime_error("Unable to load required frame index " + std::to_string(frameIndex));
}
} // namespace

auto loadViewportMetadata(const Common::Json &config, const Placeholders &placeholders,
                          int32_t frameIndex, const std::string &cameraName, bool isPoseTrace)
    -> MivBitstream::CameraConfig {
  const auto viewportParamsPath =
      config.require(configDirectory).as<std::filesystem::path>() /
      fmt::format(config.require(inputViewportParamsPathFmt).as<std::string>(),
                  placeholders.numberOfInputFrames, placeholders.contentId, placeholders.testId);
  std::ifstream stream{viewportParamsPath};
  if (!stream.good()) {
    throw std::runtime_error(
        fmt::format("Failed to load viewport parameters from {}", viewportParamsPath));
  }

  const auto sequenceConfig = MivBitstream::SequenceConfig{stream};
  auto result = sequenceConfig.cameraByName(isPoseTrace ? "viewport"s : cameraName);

  // The result may have invalid depth values
  result.viewParams.hasOccupancy = true;

  if (isPoseTrace) {
    const auto poseTracePath = config.require(configDirectory).as<std::filesystem::path>() /
                               fmt::format(config.require(inputPoseTracePathFmt).as<std::string>(),
                                           placeholders.numberOfInputFrames, placeholders.contentId,
                                           placeholders.testId, cameraName);
    std::ifstream stream2{poseTracePath};
    if (!stream2.good()) {
      throw std::runtime_error(
          fmt::format("Failed to load pose trace file from {}", poseTracePath));
    }

    const auto pose = loadPoseFromCSV(stream2, frameIndex);

    result.viewParams.pose.position += pose.position;
    result.viewParams.pose.orientation = eulerDeg2quat(pose.rotation);
  }

  return result;
}

namespace {
struct VideoFormat {
  Common::Vec2i frameSize;
  uint32_t bitDepth{};
  Common::ColorFormat colorFormat{};
};

template <typename Element = Common::DefaultElement>
auto loadVideoFrame(const std::string &key, const Common::Json &config,
                    const Placeholders &placeholders, MivBitstream::AtlasId atlasId,
                    int32_t frameIdx, const VideoFormat &videoFormat) -> Common::Frame<Element> {
  const auto path =
      config.require(IO::outputDirectory).as<std::filesystem::path>() /
      fmt::format(config.require(key).as<std::string>(), placeholders.numberOfInputFrames,
                  placeholders.contentId, placeholders.testId, atlasId, videoFormat.frameSize.x(),
                  videoFormat.frameSize.y());

  return IO::loadFrame<Element>(path, frameIdx, videoFormat.frameSize, videoFormat.bitDepth,
                                videoFormat.colorFormat);
}
} // namespace

auto loadOccupancyVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                             MivBitstream::AtlasId atlasId, int32_t frameIdx,
                             Common::Vec2i frameSize, uint32_t bitDepth)
    -> Common::Occupancy10Frame {
  return loadVideoFrame<>(IO::inputOccupancyVideoFramePathFmt, config, placeholders, atlasId,
                          frameIdx, {frameSize, bitDepth, Common::ColorFormat::YUV400});
}

auto loadGeometryVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                            MivBitstream::AtlasId atlasId, int32_t frameIdx,
                            Common::Vec2i frameSize, uint32_t bitDepth) -> Common::Depth10Frame {
  return loadVideoFrame<>(IO::inputGeometryVideoFramePathFmt, config, placeholders, atlasId,
                          frameIdx, {frameSize, bitDepth, Common::ColorFormat::YUV400});
}

auto loadTextureVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                           MivBitstream::AtlasId atlasId, int32_t frameIdx, Common::Vec2i frameSize,
                           uint32_t bitDepth) -> Common::Texture444Frame {
  return loadVideoFrame<>(IO::inputTextureVideoFramePathFmt, config, placeholders, atlasId,
                          frameIdx, {frameSize, bitDepth, Common::ColorFormat::YUV420});
}

auto loadTransparencyVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                                MivBitstream::AtlasId atlasId, int32_t frameIdx,
                                Common::Vec2i frameSize, uint32_t bitDepth)
    -> Common::Transparency10Frame {
  return loadVideoFrame<>(IO::inputTransparencyVideoFramePathFmt, config, placeholders, atlasId,
                          frameIdx, {frameSize, bitDepth, Common::ColorFormat::YUV400});
}

auto loadFramePackVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                             MivBitstream::AtlasId atlasId, int32_t frameIdx,
                             Common::Vec2i frameSize, uint32_t bitDepth)
    -> Common::FramePack444Frame {
  return loadVideoFrame<>(IO::inputPackedVideoFramePathFmt, config, placeholders, atlasId, frameIdx,
                          {frameSize, bitDepth, Common::ColorFormat::YUV420});
}

namespace {
template <bool allowNullopt>
auto tryLoadSequenceConfig_(const Common::Json &config, const Placeholders &placeholders,
                            int32_t frameIndex)
    -> std::conditional_t<allowNullopt, std::optional<MivBitstream::SequenceConfig>,
                          MivBitstream::SequenceConfig> {
  const auto relPath = fmt::format(config.require(IO::inputSequenceConfigPathFmt).as<std::string>(),
                                   placeholders.numberOfInputFrames, placeholders.contentId,
                                   placeholders.testId, frameIndex);
  const auto path1 = config.require(IO::inputDirectory).as<std::filesystem::path>() / relPath;
  const auto path2 = config.require(IO::configDirectory).as<std::filesystem::path>() / relPath;
  const auto path = exists(path1) ? path1 : path2;

  if constexpr (allowNullopt) {
    if (!exists(path)) {
      return std::nullopt;
    }
  }
  std::ifstream stream{path};
  if (!stream.good()) {
    throw std::runtime_error(
        fmt::format("Failed to load source camera parameters from {} or {} (with current path {})",
                    path1, path2, std::filesystem::current_path()));
  }
  return MivBitstream::SequenceConfig{stream};
}
} // namespace

auto loadSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                        int32_t frameIndex) -> MivBitstream::SequenceConfig {
  return tryLoadSequenceConfig_<false>(config, placeholders, frameIndex);
}

auto tryLoadSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                           int32_t frameIndex) -> std::optional<MivBitstream::SequenceConfig> {
  return tryLoadSequenceConfig_<true>(config, placeholders, frameIndex);
}

auto inputBitstreamPath(const Common::Json &config, const Placeholders &placeholders)
    -> std::filesystem::path {
  return config.require(IO::inputDirectory).as<std::filesystem::path>() /
         fmt::format(config.require(IO::inputBitstreamPathFmt).as<std::string>(),
                     placeholders.numberOfInputFrames, placeholders.contentId, placeholders.testId);
}

auto inputSubBitstreamPath(const std::string &key, const Common::Json &config,
                           const Placeholders &placeholders, MivBitstream::AtlasId atlasId,
                           int32_t attributeIdx) -> std::filesystem::path {
  return config.require(IO::inputDirectory).as<std::filesystem::path>() /
         fmt::format(config.require(key).as<std::string>(), placeholders.numberOfInputFrames,
                     placeholders.contentId, placeholders.testId, atlasId, attributeIdx);
}
} // namespace TMIV::IO
