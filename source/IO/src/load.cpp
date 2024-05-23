/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#include "DependencyInjector.h"

#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/MivBitstream/Formatters.h>

#include <fstream>
#include <regex>

using namespace std::string_literals;

namespace TMIV::IO {
template <typename Element = Common::DefaultElement>
auto loadFrame(const std::filesystem::path &path, int32_t frameIdx, Common::Vec2i frameSize,
               uint32_t bitDepth, Common::ColorFormat colorFormat) -> Common::Frame<Element> {
  return Common::withElement(bitDepth, [&](auto zero) {
    using NativeElement = decltype(zero);

    if constexpr (std::is_same_v<NativeElement, Element>) {
      auto frame = Common::Frame<Element>{frameSize, bitDepth, colorFormat};

      auto &filesystem = DependencyInjector::getInstance().filesystem();
      auto stream_ = filesystem.ifstream(path, std::ios::binary);
      auto &stream = *stream_;

      if (!stream.good()) {
        throw std::runtime_error(TMIV_FMT::format("Failed to open {} for reading", path));
      }

      stream.seekg(std::streampos(frameIdx) * frame.getByteCount());

      if (!stream.good()) {
        throw std::runtime_error(
            TMIV_FMT::format("Failed to seek for reading to frame {} from {}", frameIdx, path));
      }

      frame.readFrom(stream);

      if (!stream.good()) {
        throw std::runtime_error(
            TMIV_FMT::format("Failed to read frame {} from {}", frameIdx, path));
      }

      return frame;
    } else {
      return Common::elementCast<Element>(
          loadFrame<NativeElement>(path, frameIdx, frameSize, bitDepth, colorFormat));
    }
  });
}

namespace {
struct LoadInputFrameParams {
  bool isOptional{};
  const Common::Json &config;
  const Placeholders &placeholders;
  int32_t frameIdx{};
  Common::Vec2i frameSize;
  std::string name;
};

auto loadInputFrame(const LoadInputFrameParams &params, Common::ColorFormat colorFormat,
                    uint32_t bitDepth, MivBitstream::VuhUnitType vuhUnitType,
                    MivBitstream::AiAttributeTypeId attrTypeId =
                        MivBitstream::AiAttributeTypeId::ATTR_UNSPECIFIED) -> Common::Frame<> {
  const auto configKey =
      TMIV_FMT::format("input{}PathFmt", videoComponentName(vuhUnitType, attrTypeId));

  if (const auto &node = params.config.optional(configKey)) {
    const auto inputDir = params.config.require("inputDirectory").as<std::filesystem::path>();
    const auto videoFormat = videoFormatString(colorFormat, bitDepth);

    const auto path =
        inputDir /
        Common::runtimeFormat(node.as<std::string>(), params.placeholders.numberOfInputFrames,
                              params.placeholders.contentId, params.placeholders.testId,
                              params.name, params.frameSize.x(), params.frameSize.y(), videoFormat);

    return loadFrame<>(path, params.placeholders.startFrame + params.frameIdx, params.frameSize,
                       bitDepth, colorFormat);
  }
  if (params.isOptional) {
    return {};
  }
  throw std::runtime_error(
      TMIV_FMT::format("The required configuration key {} is missing", configKey));
}

auto loadEntityFrame(const LoadInputFrameParams &params, Common::ColorFormat colorFormat,
                     uint32_t bitDepth) -> Common::Frame<> {
  const auto configKey = "inputEntityPathFmt"s;

  if (const auto &node = params.config.optional(configKey)) {
    const auto inputDir = params.config.require("inputDirectory").as<std::filesystem::path>();
    const auto videoFormat = videoFormatString(colorFormat, bitDepth);

    const auto path =
        inputDir /
        Common::runtimeFormat(node.as<std::string>(), params.placeholders.numberOfInputFrames,
                              params.placeholders.contentId, params.placeholders.testId,
                              params.name, params.frameSize.x(), params.frameSize.y(), videoFormat);

    return loadFrame<>(path, params.placeholders.startFrame + params.frameIdx, params.frameSize,
                       bitDepth, colorFormat);
  }
  LIMITATION(params.isOptional);
  return {};
}

auto requantize(Common::Frame<> frame) {
  if (frame.empty()) {
    return frame;
  }

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

void touchLoadMultiviewFrameKeys(const Common::Json &config) {
  using VUT = MivBitstream::VuhUnitType;
  using ATI = MivBitstream::AiAttributeTypeId;

  config.require("inputDirectory");
  config.optional(TMIV_FMT::format("input{}PathFmt", videoComponentName(VUT::V3C_GVD)));
  config.optional(
      TMIV_FMT::format("input{}PathFmt", videoComponentName(VUT::V3C_AVD, ATI::ATTR_TEXTURE)));
  config.optional(
      TMIV_FMT::format("input{}PathFmt", videoComponentName(VUT::V3C_AVD, ATI::ATTR_TRANSPARENCY)));
  config.optional("inputEntityPathFmt");
}

auto loadMultiviewFrame(const Common::Json &config, const Placeholders &placeholders,
                        const MivBitstream::SequenceConfig &sc, int32_t frameIdx)
    -> Common::DeepFrameList {
  auto frame = Common::DeepFrameList(sc.sourceCameraNames.size());

  const auto inputDir = config.require("inputDirectory").as<std::filesystem::path>();
  const auto startFrame = placeholders.startFrame;
  Common::logInfo("Loading multiview frame {0} with start frame offset {1} (= {2}).", frameIdx,
                  startFrame, frameIdx + startFrame);

  for (size_t v = 0; v < frame.size(); ++v) {
    const auto &name = sc.sourceCameraNames[v];
    const auto camera = sc.cameraByName(name);

    const auto params = LoadInputFrameParams{
        true, config, placeholders, frameIdx, camera.viewParams.ci.projectionPlaneSize(), name};

    frame[v].texture = yuv420(loadInputFrame(
        params, camera.colorFormatTexture, camera.bitDepthTexture,
        MivBitstream::VuhUnitType::V3C_AVD, MivBitstream::AiAttributeTypeId::ATTR_TEXTURE));

    frame[v].transparency = yuv400(loadInputFrame(
        params, camera.colorFormatTransparency, camera.bitDepthTransparency,
        MivBitstream::VuhUnitType::V3C_AVD, MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY));

    frame[v].geometry = requantize(
        yuv400(loadInputFrame(params, camera.colorFormatGeometry, camera.bitDepthGeometry,
                              MivBitstream::VuhUnitType::V3C_GVD)));

    frame[v].entities =
        yuv400(loadEntityFrame(params, camera.colorFormatEntities, camera.bitDepthEntities));
  }

  return frame;
}

auto loadMpiTextureMpiLayer(const Common::Json &config, const Placeholders &placeholders,
                            const MivBitstream::SequenceConfig &sc, int32_t frameIdx,
                            int32_t mpiLayerIdx, int32_t nbMpiLayers) -> Common::Frame<> {
  const auto name = sc.sourceCameraNames[0];
  const auto camera = sc.cameraByName(name);

  return loadInputFrame(
      LoadInputFrameParams{false, config, placeholders, frameIdx * nbMpiLayers + mpiLayerIdx,
                           camera.viewParams.ci.projectionPlaneSize(), name},
      camera.colorFormatTexture, camera.bitDepthTexture, MivBitstream::VuhUnitType::V3C_AVD,
      MivBitstream::AiAttributeTypeId::ATTR_TEXTURE);
}

auto loadMpiTransparencyMpiLayer(const Common::Json &config, const Placeholders &placeholders,
                                 const MivBitstream::SequenceConfig &sc, int32_t frameIdx,
                                 int32_t mpiLayerIdx, int32_t nbMpiLayers) -> Common::Frame<> {
  const auto name = sc.sourceCameraNames[0];
  const auto camera = sc.cameraByName(name);

  return loadInputFrame(
      LoadInputFrameParams{false, config, placeholders, frameIdx * nbMpiLayers + mpiLayerIdx,
                           camera.viewParams.ci.projectionPlaneSize(), name},
      camera.colorFormatTransparency, camera.bitDepthTransparency,
      MivBitstream::VuhUnitType::V3C_AVD, MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY);
}

namespace {
struct Pose {
  Common::Vec3f position;
  Common::Vec3f rotation;
};

auto loadPoseFromCSV(std::istream &stream, int32_t frameIdx) -> Pose {
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
      if (currentFrameIndex == frameIdx) {
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

  throw std::runtime_error("Unable to load required frame index " + std::to_string(frameIdx));
}
} // namespace

auto loadViewportMetadata(const Common::Json &config, const Placeholders &placeholders,
                          int32_t frameIdx, const std::string &cameraName, bool isPoseTrace)
    -> MivBitstream::CameraConfig {
  const auto viewportParamsPath =
      config.require("configDirectory").as<std::filesystem::path>() /
      Common::runtimeFormat(config.require("inputViewportParamsPathFmt").as<std::string>(),
                            placeholders.numberOfInputFrames, placeholders.contentId,
                            placeholders.testId);

  auto &filesystem = DependencyInjector::getInstance().filesystem();
  auto stream = filesystem.ifstream(viewportParamsPath);

  if (!stream->good()) {
    throw std::runtime_error(
        TMIV_FMT::format("Failed to load viewport parameters from {}", viewportParamsPath));
  }

  const auto sequenceConfig = MivBitstream::SequenceConfig{*stream};
  auto result = sequenceConfig.cameraByName(isPoseTrace ? "viewport"s : cameraName);

  // The result may have invalid depth values
  result.viewParams.hasOccupancy = true;

  if (isPoseTrace) {
    const auto poseTracePath =
        config.require("configDirectory").as<std::filesystem::path>() /
        Common::runtimeFormat(config.require("inputPoseTracePathFmt").as<std::string>(),
                              placeholders.numberOfInputFrames, placeholders.contentId,
                              placeholders.testId, cameraName);
    auto stream2 = filesystem.ifstream(poseTracePath);
    if (!stream2->good()) {
      throw std::runtime_error(
          TMIV_FMT::format("Failed to load pose trace file from {}", poseTracePath));
    }

    const auto pose = loadPoseFromCSV(*stream2, frameIdx);

    result.viewParams.pose.position += pose.position;
    result.viewParams.pose.orientation = eulerDeg2quat(pose.rotation);
  }

  return result;
}

void touchLoadViewportMetadataKeys(const Common::Json &config) {
  config.require("configDirectory");
  config.require("inputViewportParamsPathFmt");
  config.optional("inputPoseTracePathFmt");
}

namespace {
auto loadVuhFromJson(const Common::Json &node) -> MivBitstream::V3cUnitHeader {
  const auto vuh_unit_type = node.require("vuh_unit_type").as<MivBitstream::VuhUnitType>();
  const auto vuh_v3c_parameter_set_id = node.require("vuh_v3c_parameter_set_id").as<uint8_t>();
  const auto vuh_atlas_id = MivBitstream::AtlasId{node.require("vuh_atlas_id").as<uint8_t>()};

  switch (vuh_unit_type) {
  case MivBitstream::VuhUnitType::V3C_OVD:
    return MivBitstream::V3cUnitHeader::ovd(vuh_v3c_parameter_set_id, vuh_atlas_id);
  case MivBitstream::VuhUnitType::V3C_GVD:
    return MivBitstream::V3cUnitHeader::gvd(vuh_v3c_parameter_set_id, vuh_atlas_id,
                                            node.require("vuh_map_index").as<uint8_t>(),
                                            node.require("vuh_auxiliary_video_flag").as<bool>());
  case MivBitstream::VuhUnitType::V3C_AVD:
    return MivBitstream::V3cUnitHeader::avd(
        vuh_v3c_parameter_set_id, vuh_atlas_id, node.require("vuh_attribute_index").as<uint8_t>(),
        node.require("vuh_attribute_partition_index").as<uint8_t>(),
        node.require("vuh_map_index").as<uint8_t>(),
        node.require("vuh_auxiliary_video_flag").as<bool>());
  case MivBitstream::VuhUnitType::V3C_PVD:
    return MivBitstream::V3cUnitHeader::pvd(vuh_v3c_parameter_set_id, vuh_atlas_id);

  default:
    throw std::runtime_error(TMIV_FMT::format("Invalid V3C unit type ID {}", vuh_unit_type));
  }
}

struct OutOfBandMetadata {
  explicit OutOfBandMetadata(const Common::Json &node)
      : vuh(loadVuhFromJson(node))
      , attrTypeId{vuh.vuh_unit_type() == MivBitstream::VuhUnitType::V3C_AVD
                       ? node.require("ai_attribute_type_id").as<MivBitstream::AiAttributeTypeId>()
                       : MivBitstream::AiAttributeTypeId::ATTR_UNSPECIFIED}
      , frameSize{node.require("frame_size").asVec<int32_t, 2>()}
      , bitDepth{node.require("bit_depth").as<uint32_t>()}
      , irapFrameIndices{node.require("irap_frame_indices").asVector<int32_t>()} {}

  MivBitstream::V3cUnitHeader vuh{};
  MivBitstream::AiAttributeTypeId attrTypeId{};
  Common::Vec2i frameSize{};
  uint32_t bitDepth{};
  Common::ColorFormat colorFormat{Common::ColorFormat::YUV420};
  std::vector<int32_t> irapFrameIndices;
};

auto outOfBandMetadataPath(const Common::Json &config, const Placeholders &placeholders) {
  return inputBitstreamPath(config, placeholders).replace_extension(".json");
}

auto loadOutOfbandMetadata(const Common::Json &config, const Placeholders &placeholders,
                           MivBitstream::V3cUnitHeader vuh) {
  const auto file = outOfBandMetadataPath(config, placeholders);
  auto &filesystem = DependencyInjector::getInstance().filesystem();
  auto stream = filesystem.ifstream(file);

  if (!stream->good()) {
    throw std::runtime_error(
        TMIV_FMT::format("Failed to load the out-of-band metadata from {}", file));
  }

  const auto metadata = Common::Json::loadFrom(*stream);

  for (const auto &node : metadata.as<Common::Json::Array>()) {
    if (loadVuhFromJson(node) == vuh) {
      return OutOfBandMetadata{node};
    }
  }

  throw std::runtime_error(
      TMIV_FMT::format("Missing V3C unit header in out-of-band metadata file:\n{}", vuh));
}
} // namespace

auto loadOutOfBandVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                             MivBitstream::V3cUnitHeader vuh, int32_t frameIdx)
    -> Common::DecodedFrame {
  if (placeholders.numberOfInputFrames <= frameIdx) {
    return {};
  }

  const auto metadata = loadOutOfbandMetadata(config, placeholders, vuh);
  const auto configKey = TMIV_FMT::format(
      "input{}VideoFramePathFmt", videoComponentName(vuh.vuh_unit_type(), metadata.attrTypeId));
  const auto videoFormat = videoFormatString(metadata.colorFormat, metadata.bitDepth);

  const auto path =
      config.require("inputDirectory").as<std::filesystem::path>() /
      Common::runtimeFormat(config.require(configKey).as<std::string>(),
                            placeholders.numberOfInputFrames, placeholders.contentId,
                            placeholders.testId, vuh.vuh_atlas_id().asInt(), metadata.frameSize.x(),
                            metadata.frameSize.y(), videoFormat);

  return {loadFrame<>(path, frameIdx, metadata.frameSize, metadata.bitDepth, metadata.colorFormat),
          Common::contains(metadata.irapFrameIndices, frameIdx)};
}

void touchLoadOutOfBandVideoFrameKeys(const Common::Json &config) {
  config.require("inputDirectory");
  config.require("inputBitstreamPathFmt");
  config.optional("inputOccupancyVideoFramePathFmt");
  config.optional("inputGeometryVideoFramePathFmt");
  config.optional("inputPackedVideoFramePathFmt");
  config.optional("inputTextureVideoFramePathFmt");
  config.optional("inputTransparencyVideoFramePathFmt");
  config.optional("inputMaterialIdVideoFramePathFmt");
  config.optional("inputReflectanceVideoFramePathFmt");
  config.optional("inputNormalVideoFramePathFmt");
}

namespace {
template <bool allowNullopt>
auto tryLoadSequenceConfig_(const Common::Json &config, const Placeholders &placeholders,
                            int32_t frameIdx)
    -> std::conditional_t<allowNullopt, std::optional<MivBitstream::SequenceConfig>,
                          MivBitstream::SequenceConfig> {
  auto &filesystem = DependencyInjector::getInstance().filesystem();

  const auto relPath = Common::runtimeFormat(
      config.require("inputSequenceConfigPathFmt").as<std::string>(),
      placeholders.numberOfInputFrames, placeholders.contentId, placeholders.testId, frameIdx);
  const auto path1 = config.require("inputDirectory").as<std::filesystem::path>() / relPath;
  const auto path2 = config.require("configDirectory").as<std::filesystem::path>() / relPath;
  const auto path = filesystem.exists(path1) ? path1 : path2;

  if (!filesystem.exists(path)) {
    if constexpr (allowNullopt) {
      return std::nullopt;
    } else {
      throw std::runtime_error(
          TMIV_FMT::format("Source camera parameter paths {} and {} do not exist", path1, path2));
    }
  }

  auto stream = filesystem.ifstream(path);

  if (!stream->good()) {
    throw std::runtime_error(
        TMIV_FMT::format("Failed to load source camera parameters from {}", path));
  }

  return MivBitstream::SequenceConfig{*stream};
}
} // namespace

auto loadSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                        int32_t frameIdx) -> MivBitstream::SequenceConfig {
  return tryLoadSequenceConfig_<false>(config, placeholders, frameIdx);
}

auto tryLoadSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                           int32_t frameIdx) -> std::optional<MivBitstream::SequenceConfig> {
  return tryLoadSequenceConfig_<true>(config, placeholders, frameIdx);
}

void touchLoadSequenceConfigKeys(const Common::Json &config) {
  config.require("inputSequenceConfigPathFmt");
  config.require("inputDirectory");
  config.require("configDirectory");
}

auto inputBitstreamPath(const Common::Json &config, const Placeholders &placeholders)
    -> std::filesystem::path {
  return config.require("inputDirectory").as<std::filesystem::path>() /
         Common::runtimeFormat(config.require("inputBitstreamPathFmt").as<std::string>(),
                               placeholders.numberOfInputFrames, placeholders.contentId,
                               placeholders.testId);
}

void touchInputBitstreamPathKeys(const Common::Json &config) {
  config.require("inputDirectory");
  config.require("inputBitstreamPathFmt");
}

auto inputVideoSubBitstreamPath(const Common::Json &config, const Placeholders &placeholders,
                                MivBitstream::V3cUnitHeader vuh,
                                MivBitstream::AiAttributeTypeId attrTypeId)
    -> std::filesystem::path {
  const auto configKey = TMIV_FMT::format("input{}VideoSubBitstreamPathFmt",
                                          videoComponentName(vuh.vuh_unit_type(), attrTypeId));

  const auto attrIdx = vuh.vuh_unit_type() == MivBitstream::VuhUnitType::V3C_AVD
                           ? vuh.vuh_attribute_index()
                           : uint8_t{};

  return config.require("inputDirectory").as<std::filesystem::path>() /
         Common::runtimeFormat(config.require(configKey).as<std::string>(),
                               placeholders.numberOfInputFrames, placeholders.contentId,
                               placeholders.testId, vuh.vuh_atlas_id().asInt(), attrIdx);
}

void touchInputVideoSubBitstreamPathKeys(const Common::Json &config) {
  config.require("inputDirectory");
  config.optional("inputOccupancyVideoSubBitstreamPathFmt");
  config.optional("inputGeometryVideoSubBitstreamPathFmt");
  config.optional("inputPackedVideoSubBitstreamPathFmt");
  config.optional("inputTextureVideoSubBitstreamPathFmt");
  config.optional("inputTransparencyVideoSubBitstreamPathFmt");
  config.optional("inputMaterialIdVideoSubBitstreamPathFmt");
  config.optional("inputReflectanceVideoSubBitstreamPathFmt");
  config.optional("inputNormalVideoSubBitstreamPathFmt");
}
} // namespace TMIV::IO
