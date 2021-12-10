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

#include "impl.hpp"

using namespace std::string_literals;

namespace TMIV::IO {
template <typename Element = Common::DefaultElement>
auto loadFrame(const std::filesystem::path &path, int32_t frameIdx, Common::Vec2i frameSize,
               uint32_t bitDepth, Common::ColorFormat colorFormat) -> Common::Frame<Element> {
  return Common::withElement(bitDepth, [&](auto zero) -> Common::Frame<Element> {
    using FileElement = decltype(zero);

    auto frame = Common::Frame<FileElement>{frameSize, bitDepth, colorFormat};

    std::ifstream stream{path, std::ios::binary};
    if (!stream.good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for reading", path));
    }

    stream.seekg(std::streampos(frameIdx) * frame.getDiskSize());
    if (!stream.good()) {
      throw std::runtime_error(
          fmt::format("Failed to seek for reading to frame {} from {}", frameIdx, path));
    }

    frame.read(stream);
    if (!stream.good()) {
      throw std::runtime_error(fmt::format("Failed to read frame {} from {}", frameIdx, path));
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
                        const MivBitstream::SequenceConfig &sc, int32_t frameIdx)
    -> Common::MVD16Frame {
  auto frame = Common::MVD16Frame(sc.sourceCameraNames.size());

  const auto inputDir = config.require("inputDirectory").as<std::filesystem::path>();
  const auto startFrame = placeholders.startFrame;
  fmt::print("Loading multiview frame {0} with start frame offset {1} (= {2}).\n", frameIdx,
             startFrame, frameIdx + startFrame);

  // TODO(#397): Generalize Common::TextureDepth16Frame to handle all attributes

  for (size_t v = 0; v < frame.size(); ++v) {
    const auto name = sc.sourceCameraNames[v];
    const auto camera = sc.cameraByName(name);
    const auto &vp = camera.viewParams;
    const auto frameSize = vp.ci.projectionPlaneSize();

    if (const auto &node = config.optional("inputTexturePathFmt")) {
      const auto path =
          inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                 placeholders.contentId, placeholders.testId, name, frameSize.x(),
                                 frameSize.y(), camera.textureVideoFormat());
      frame[v].texture = loadFrame<>(path, startFrame + frameIdx, frameSize, camera.bitDepthColor,
                                     Common::ColorFormat::YUV420);
    }

    if (const auto &node = config.optional("inputTransparencyPathFmt")) {
      const auto path =
          inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                 placeholders.contentId, placeholders.testId, name, frameSize.x(),
                                 frameSize.y(), camera.transparencyVideoFormat());
      frame[v].transparency = loadFrame<>(path, startFrame + frameIdx, frameSize,
                                          camera.bitDepthTransparency, Common::ColorFormat::YUV400);
    }

    if (const auto &node = config.optional("inputGeometryPathFmt")) {
      const auto path =
          inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                 placeholders.contentId, placeholders.testId, name, frameSize.x(),
                                 frameSize.y(), camera.geometryVideoFormat());
      frame[v].depth = requantize(loadFrame<>(path, startFrame + frameIdx, frameSize,
                                              camera.bitDepthDepth, Common::ColorFormat::YUV400));
    }

    if (const auto &node = config.optional("inputEntityPathFmt")) {
      const auto path =
          inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                 placeholders.contentId, placeholders.testId, name, frameSize.x(),
                                 frameSize.y(), camera.entitiesVideoFormat());
      frame[v].entities = loadFrame<>(path, startFrame + frameIdx, frameSize,
                                      camera.bitDepthEntities, Common::ColorFormat::YUV400);
    }
  }

  return frame;
}

auto loadMpiTextureMpiLayer(const Common::Json &config, const Placeholders &placeholders,
                            const MivBitstream::SequenceConfig &sc, int32_t frameIdx,
                            int32_t mpiLayerIndex, int32_t nbMpiLayers) -> Common::TextureFrame {
  const auto inputDir = config.require("inputDirectory").as<std::filesystem::path>();
  const auto &node = config.require("inputTexturePathFmt");

  const auto name = sc.sourceCameraNames[0];

  const auto camera = sc.cameraByName(name);
  const auto &vp = camera.viewParams;
  const auto frameSize = vp.ci.projectionPlaneSize();

  const auto path = inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                           placeholders.contentId, placeholders.testId, name,
                                           frameSize.x(), frameSize.y(), "yuv420p10le");

  auto texture = loadFrame<>(path, frameIdx * nbMpiLayers + mpiLayerIndex, frameSize,
                             camera.bitDepthColor, Common::ColorFormat::YUV420);

  return texture;
}

auto loadMpiTransparencyMpiLayer(const Common::Json &config, const Placeholders &placeholders,
                                 const MivBitstream::SequenceConfig &sc, int32_t frameIdx,
                                 int32_t mpiLayerIndex, int32_t nbMpiLayers)
    -> Common::Transparency8Frame {
  const auto inputDir = config.require("inputDirectory").as<std::filesystem::path>();
  const auto &node = config.require("inputTransparencyPathFmt");

  const auto name = sc.sourceCameraNames[0];

  const auto camera = sc.cameraByName(name);
  const auto &vp = camera.viewParams;
  const auto frameSize = vp.ci.projectionPlaneSize();

  const auto path = inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                           placeholders.contentId, placeholders.testId, name,
                                           frameSize.x(), frameSize.y(), "yuv420p");

  return loadFrame<uint8_t>(path, frameIdx * nbMpiLayers + mpiLayerIndex, frameSize,
                            camera.bitDepthTransparency, Common::ColorFormat::YUV400);
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
      fmt::format(config.require("inputViewportParamsPathFmt").as<std::string>(),
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
    const auto poseTracePath =
        config.require("configDirectory").as<std::filesystem::path>() /
        fmt::format(config.require("inputPoseTracePathFmt").as<std::string>(),
                    placeholders.numberOfInputFrames, placeholders.contentId, placeholders.testId,
                    cameraName);
    std::ifstream stream2{poseTracePath};
    if (!stream2.good()) {
      throw std::runtime_error(
          fmt::format("Failed to load pose trace file from {}", poseTracePath));
    }

    const auto pose = loadPoseFromCSV(stream2, frameIdx);

    result.viewParams.pose.position += pose.position;
    result.viewParams.pose.orientation = eulerDeg2quat(pose.rotation);
  }

  return result;
}

namespace {
[[nodiscard]] auto attrTypeId(MivBitstream::V3cUnitHeader vuh,
                              const MivBitstream::V3cParameterSet &vps) noexcept {
  if (vuh.vuh_unit_type() == MivBitstream::VuhUnitType::V3C_AVD) {
    const auto &ai = vps.attribute_information(vuh.vuh_atlas_id());
    return ai.ai_attribute_type_id(vuh.vuh_attribute_index());
  }
  return MivBitstream::AiAttributeTypeId::ATTR_UNSPECIFIED;
}

[[nodiscard]] auto outOfBandFrameSizeOf(MivBitstream::V3cUnitHeader vuh,
                                        const MivBitstream::V3cParameterSet &vps,
                                        const MivBitstream::AtlasSequenceParameterSetRBSP &asps)
    -> Common::Vec2i {
  auto scaleFactor = Common::Vec2i{1, 1};

  switch (vuh.vuh_unit_type()) {
  case MivBitstream::VuhUnitType::V3C_OVD:
    if (asps.asps_miv_extension_present_flag() &&
        asps.asps_miv_extension().asme_occupancy_scale_enabled_flag()) {
      scaleFactor = {asps.asps_miv_extension().asme_occupancy_scale_factor_x_minus1() + 1,
                     asps.asps_miv_extension().asme_occupancy_scale_factor_y_minus1() + 1};
    }
    break;
  case MivBitstream::VuhUnitType::V3C_GVD:
    if (asps.asps_miv_extension_present_flag() &&
        asps.asps_miv_extension().asme_geometry_scale_enabled_flag()) {
      scaleFactor = {asps.asps_miv_extension().asme_geometry_scale_factor_x_minus1() + 1,
                     asps.asps_miv_extension().asme_geometry_scale_factor_y_minus1() + 1};
    }
    break;
  case MivBitstream::VuhUnitType::V3C_AVD:
    break;
  case MivBitstream::VuhUnitType::V3C_PVD: {
    auto size = Common::Vec2i{};

    const auto &pin = vps.packing_information(vuh.vuh_atlas_id());

    for (uint8_t regionIdx = 0; regionIdx <= pin.pin_regions_count_minus1(); ++regionIdx) {
      size.x() = std::max(pin.pin_region_top_left_x(regionIdx) +
                              pin.pin_region_width_minus1(regionIdx) + 1,
                          size.x());
      size.y() = std::max(pin.pin_region_top_left_y(regionIdx) +
                              pin.pin_region_height_minus1(regionIdx) + 1,
                          size.y());
    }

    return size;
  }
  default:
    UNREACHABLE;
  }

  return {vps.vps_frame_width(vuh.vuh_atlas_id()) / scaleFactor.x(),
          vps.vps_frame_height(vuh.vuh_atlas_id()) / scaleFactor.y()};
}

[[nodiscard]] auto outOfBandBitDepthOf(MivBitstream::V3cUnitHeader vuh,
                                       const MivBitstream::V3cParameterSet &vps) noexcept {
  switch (vuh.vuh_unit_type()) {
  case MivBitstream::VuhUnitType::V3C_OVD: {
    const auto &oi = vps.occupancy_information(vuh.vuh_atlas_id());
    return oi.oi_occupancy_2d_bit_depth_minus1() + 1U;
  }
  case MivBitstream::VuhUnitType::V3C_GVD: {
    const auto &gi = vps.geometry_information(vuh.vuh_atlas_id());
    return gi.gi_geometry_2d_bit_depth_minus1() + 1U;
  }
  case MivBitstream::VuhUnitType::V3C_AVD: {
    const auto &ai = vps.attribute_information(vuh.vuh_atlas_id());
    return ai.ai_attribute_2d_bit_depth_minus1(vuh.vuh_attribute_index()) + 1U;
  }
  case MivBitstream::VuhUnitType::V3C_PVD: {
    auto bitDepth = 0U;

    const auto &pin = vps.packing_information(vuh.vuh_atlas_id());

    if (pin.pin_occupancy_present_flag()) {
      bitDepth = std::max(bitDepth, pin.pin_occupancy_2d_bit_depth_minus1() + 1U);
    }

    if (pin.pin_geometry_present_flag()) {
      bitDepth = std::max(bitDepth, pin.pin_geometry_2d_bit_depth_minus1() + 1U);
    }

    if (pin.pin_attribute_present_flag()) {
      for (uint8_t i = 0; i < pin.pin_attribute_count(); ++i) {
        bitDepth = std::max(bitDepth, pin.pin_attribute_2d_bit_depth_minus1(i) + 1U);
      }
    }

    return bitDepth;
  }
  default:
    UNREACHABLE;
  }
}
} // namespace

auto loadOutOfBandVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                             MivBitstream::V3cUnitHeader vuh, int32_t frameIdx,
                             const MivBitstream::V3cParameterSet &vps,
                             const MivBitstream::AtlasSequenceParameterSetRBSP &asps)
    -> Common::Frame<> {
  if (placeholders.numberOfInputFrames <= frameIdx) {
    return Common::Frame<>{};
  }

  const auto configKey =
      fmt::format("input{}VideoFramePathFmt",
                  detail::videoComponentName(vuh.vuh_unit_type(), attrTypeId(vuh, vps)));

  const auto frameSize = outOfBandFrameSizeOf(vuh, vps, asps);
  const auto bitDepth = outOfBandBitDepthOf(vuh, vps);
  static constexpr auto colorFormat = Common::ColorFormat::YUV420;

  const auto path =
      config.require("outputDirectory").as<std::filesystem::path>() /
      fmt::format(config.require(configKey).as<std::string>(), placeholders.numberOfInputFrames,
                  placeholders.contentId, placeholders.testId, vuh.vuh_atlas_id(), frameSize.x(),
                  frameSize.y());

  return IO::loadFrame<>(path, frameIdx, frameSize, bitDepth, colorFormat);
}

namespace {
template <bool allowNullopt>
auto tryLoadSequenceConfig_(const Common::Json &config, const Placeholders &placeholders,
                            int32_t frameIdx)
    -> std::conditional_t<allowNullopt, std::optional<MivBitstream::SequenceConfig>,
                          MivBitstream::SequenceConfig> {
  const auto relPath = fmt::format(config.require("inputSequenceConfigPathFmt").as<std::string>(),
                                   placeholders.numberOfInputFrames, placeholders.contentId,
                                   placeholders.testId, frameIdx);
  const auto path1 = config.require("inputDirectory").as<std::filesystem::path>() / relPath;
  const auto path2 = config.require("configDirectory").as<std::filesystem::path>() / relPath;
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
                        int32_t frameIdx) -> MivBitstream::SequenceConfig {
  return tryLoadSequenceConfig_<false>(config, placeholders, frameIdx);
}

auto tryLoadSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                           int32_t frameIdx) -> std::optional<MivBitstream::SequenceConfig> {
  return tryLoadSequenceConfig_<true>(config, placeholders, frameIdx);
}

auto inputBitstreamPath(const Common::Json &config, const Placeholders &placeholders)
    -> std::filesystem::path {
  return config.require("inputDirectory").as<std::filesystem::path>() /
         fmt::format(config.require("inputBitstreamPathFmt").as<std::string>(),
                     placeholders.numberOfInputFrames, placeholders.contentId, placeholders.testId);
}

auto inputVideoSubBitstreamPath(const Common::Json &config, const Placeholders &placeholders,
                                MivBitstream::V3cUnitHeader vuh,
                                MivBitstream::AiAttributeTypeId attrTypeId)
    -> std::filesystem::path {
  const auto configKey = fmt::format("input{}VideoSubBitstreamPathFmt",
                                     detail::videoComponentName(vuh.vuh_unit_type(), attrTypeId));

  const auto attrIdx = vuh.vuh_unit_type() == MivBitstream::VuhUnitType::V3C_AVD
                           ? vuh.vuh_attribute_index()
                           : uint8_t{};

  return config.require("inputDirectory").as<std::filesystem::path>() /
         fmt::format(config.require(configKey).as<std::string>(), placeholders.numberOfInputFrames,
                     placeholders.contentId, placeholders.testId, vuh.vuh_atlas_id(), attrIdx);
}
} // namespace TMIV::IO
